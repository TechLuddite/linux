// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the gamepad MCU found in AYN Odin 2 family handhelds
 *
 * The built-in controls (buttons, sticks and analog triggers) are scanned by
 * a microcontroller that streams state reports to the SoC over a UART. The
 * rear grip buttons bypass the microcontroller and are wired to SoC GPIOs.
 * Both are reported through one input device so that the controls appear as
 * a single gamepad.
 *
 * Based on the RSInput driver by Teguh Sobirin, as carried by ROCKNIX.
 *
 * Copyright (C) 2024 Teguh Sobirin <teguh@sobir.in>
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define ODIN2_GP_BAUDRATE		115200

/*
 * Frame layout, little endian:
 *   magic[4] seq[1] cmd[1] len[2] payload[len] checksum[1]
 * The checksum is the XOR of every byte from seq up to the end of payload.
 */
#define ODIN2_GP_MAGIC			0x3d5ad3a5	/* A5 D3 5A 3D */
#define ODIN2_GP_HDR_LEN		8
#define ODIN2_GP_FRAME_MIN		(ODIN2_GP_HDR_LEN + 1)
#define ODIN2_GP_PAYLOAD_MAX		64
#define ODIN2_GP_FRAME_MAX		(ODIN2_GP_FRAME_MIN + ODIN2_GP_PAYLOAD_MAX)

#define ODIN2_GP_OFF_SEQ		4
#define ODIN2_GP_OFF_CMD		5
#define ODIN2_GP_OFF_LEN		6

#define ODIN2_GP_CMD_CONTROL		0x01
#define ODIN2_GP_CMD_STATUS		0x02

#define ODIN2_GP_CTRL_VERSION		0x02
#define ODIN2_GP_CTRL_SET_PARAMS	0x05

/* Status report payload */
#define ODIN2_GP_ST_BUTTONS		0
#define ODIN2_GP_ST_TRIGGER_L		2
#define ODIN2_GP_ST_TRIGGER_R		4
#define ODIN2_GP_ST_LX			6
#define ODIN2_GP_ST_LY			8
#define ODIN2_GP_ST_RX			10
#define ODIN2_GP_ST_RY			12
#define ODIN2_GP_ST_LEN			14

#define ODIN2_GP_STICK_MAX		0x580
#define ODIN2_GP_TRIGGER_MAX		0x610
#define ODIN2_GP_TRIGGER_FLAT		30

#define ODIN2_GP_GRIP_DEBOUNCE_MS	20

/*
 * The face buttons are labelled in the Xbox layout: Y top, X left, B right,
 * A bottom. They are reported by label, as xpad, hid-steam and hid-oxp do
 * for the same layout, because userspace (SDL in particular) maps BTN_X and
 * BTN_Y by label for every non-Sony controller. Reporting the top button as
 * BTN_NORTH (== BTN_X) swaps X and Y in every SDL program.
 */
static const unsigned int odin2_gp_keymap[] = {
	BTN_DPAD_UP,	BTN_DPAD_DOWN,	BTN_DPAD_LEFT,	BTN_DPAD_RIGHT,
	BTN_Y,		BTN_X,		BTN_B,		BTN_A,
	BTN_TL,		BTN_TR,		BTN_SELECT,	BTN_START,
	BTN_THUMBL,	BTN_THUMBR,	BTN_MODE,	BTN_BACK,
};

/* Order of the grip-gpios entries */
static const unsigned int odin2_gp_grip_keymap[] = {
	BTN_GRIPL, BTN_GRIPR,
};

struct odin2_gp {
	struct serdev_device *serdev;
	struct input_dev *input;
	struct gpio_desc *enable_gpio;
	struct gpio_descs *grips;
	struct delayed_work grip_work;
	/* Serialises reports from the UART and the grip buttons */
	struct mutex report_lock;
	u8 seq;
	size_t rx_len;
	u8 rx_buf[ODIN2_GP_FRAME_MAX];
};

static u8 odin2_gp_checksum(const u8 *frame, size_t payload_len)
{
	const u8 *p = frame + ODIN2_GP_OFF_SEQ;
	const u8 *end = frame + ODIN2_GP_HDR_LEN + payload_len;
	u8 sum = 0;

	while (p < end)
		sum ^= *p++;

	return sum;
}

static int odin2_gp_send(struct odin2_gp *gp, u8 cmd, const u8 *data, size_t len)
{
	u8 frame[ODIN2_GP_FRAME_MAX];
	size_t frame_len = ODIN2_GP_FRAME_MIN + len;
	int ret;

	if (len > ODIN2_GP_PAYLOAD_MAX)
		return -EINVAL;

	put_unaligned_le32(ODIN2_GP_MAGIC, frame);
	frame[ODIN2_GP_OFF_SEQ] = gp->seq++;
	frame[ODIN2_GP_OFF_CMD] = cmd;
	put_unaligned_le16(len, frame + ODIN2_GP_OFF_LEN);
	memcpy(frame + ODIN2_GP_HDR_LEN, data, len);
	frame[frame_len - 1] = odin2_gp_checksum(frame, len);

	ret = serdev_device_write_buf(gp->serdev, frame, frame_len);
	if (ret < 0)
		return ret;

	return ret == frame_len ? 0 : -EIO;
}

static void odin2_gp_handle_control(struct odin2_gp *gp, const u8 *data, size_t len)
{
	struct device *dev = &gp->serdev->dev;

	if (!len)
		return;

	switch (data[0]) {
	case ODIN2_GP_CTRL_VERSION:
		dev_info(dev, "MCU firmware %.*s\n", (int)(len - 1), data + 1);
		break;
	case ODIN2_GP_CTRL_SET_PARAMS:
		dev_dbg(dev, "MCU parameters acknowledged\n");
		break;
	default:
		dev_dbg(dev, "unhandled control reply 0x%02x\n", data[0]);
		break;
	}
}

static void odin2_gp_handle_status(struct odin2_gp *gp, const u8 *data, size_t len)
{
	struct input_dev *input = gp->input;
	unsigned long buttons;
	int i;

	if (len < ODIN2_GP_ST_LEN) {
		dev_warn_ratelimited(&gp->serdev->dev,
				     "short status report (%zu bytes)\n", len);
		return;
	}

	guard(mutex)(&gp->report_lock);

	buttons = get_unaligned_le16(data + ODIN2_GP_ST_BUTTONS);
	for (i = 0; i < ARRAY_SIZE(odin2_gp_keymap); i++)
		input_report_key(input, odin2_gp_keymap[i], buttons & BIT(i));

	/* Triggers report their travel counting down from the rest position */
	input_report_abs(input, ABS_Z,
			 clamp(ODIN2_GP_TRIGGER_MAX -
			       (s16)get_unaligned_le16(data + ODIN2_GP_ST_TRIGGER_L),
			       0, ODIN2_GP_TRIGGER_MAX));
	input_report_abs(input, ABS_RZ,
			 clamp(ODIN2_GP_TRIGGER_MAX -
			       (s16)get_unaligned_le16(data + ODIN2_GP_ST_TRIGGER_R),
			       0, ODIN2_GP_TRIGGER_MAX));

	/* Stick axes are reported with inverted sign */
	input_report_abs(input, ABS_X,
			 -(s16)get_unaligned_le16(data + ODIN2_GP_ST_LX));
	input_report_abs(input, ABS_Y,
			 -(s16)get_unaligned_le16(data + ODIN2_GP_ST_LY));
	input_report_abs(input, ABS_RX,
			 -(s16)get_unaligned_le16(data + ODIN2_GP_ST_RX));
	input_report_abs(input, ABS_RY,
			 -(s16)get_unaligned_le16(data + ODIN2_GP_ST_RY));

	input_sync(input);
}

static void odin2_gp_handle_frame(struct odin2_gp *gp, const u8 *frame,
				  size_t payload_len)
{
	const u8 *payload = frame + ODIN2_GP_HDR_LEN;

	switch (frame[ODIN2_GP_OFF_CMD]) {
	case ODIN2_GP_CMD_CONTROL:
		odin2_gp_handle_control(gp, payload, payload_len);
		break;
	case ODIN2_GP_CMD_STATUS:
		odin2_gp_handle_status(gp, payload, payload_len);
		break;
	default:
		dev_dbg(&gp->serdev->dev, "unhandled command 0x%02x\n",
			frame[ODIN2_GP_OFF_CMD]);
		break;
	}
}

/* Drop @n bytes from the front of the receive buffer */
static void odin2_gp_consume(struct odin2_gp *gp, size_t n)
{
	gp->rx_len -= n;
	memmove(gp->rx_buf, gp->rx_buf + n, gp->rx_len);
}

static void odin2_gp_parse(struct odin2_gp *gp)
{
	while (gp->rx_len >= ODIN2_GP_HDR_LEN) {
		size_t payload_len, frame_len;

		if (get_unaligned_le32(gp->rx_buf) != ODIN2_GP_MAGIC) {
			odin2_gp_consume(gp, 1);
			continue;
		}

		payload_len = get_unaligned_le16(gp->rx_buf + ODIN2_GP_OFF_LEN);
		if (payload_len > ODIN2_GP_PAYLOAD_MAX) {
			odin2_gp_consume(gp, 1);
			continue;
		}

		frame_len = ODIN2_GP_FRAME_MIN + payload_len;
		if (gp->rx_len < frame_len)
			return;

		if (gp->rx_buf[frame_len - 1] ==
		    odin2_gp_checksum(gp->rx_buf, payload_len)) {
			odin2_gp_handle_frame(gp, gp->rx_buf, payload_len);
			odin2_gp_consume(gp, frame_len);
		} else {
			dev_dbg_ratelimited(&gp->serdev->dev, "checksum mismatch\n");
			odin2_gp_consume(gp, 1);
		}
	}
}

static size_t odin2_gp_receive_buf(struct serdev_device *serdev,
				   const u8 *data, size_t count)
{
	struct odin2_gp *gp = serdev_device_get_drvdata(serdev);
	size_t done = 0;

	while (done < count) {
		size_t n = min(count - done, sizeof(gp->rx_buf) - gp->rx_len);

		memcpy(gp->rx_buf + gp->rx_len, data + done, n);
		gp->rx_len += n;
		done += n;

		odin2_gp_parse(gp);

		/* A full buffer that does not parse cannot hold a valid frame */
		if (gp->rx_len == sizeof(gp->rx_buf))
			odin2_gp_consume(gp, 1);
	}

	return count;
}

static const struct serdev_device_ops odin2_gp_serdev_ops = {
	.receive_buf = odin2_gp_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

static int odin2_gp_mcu_init(struct odin2_gp *gp)
{
	static const u8 version_req[] = { ODIN2_GP_CTRL_VERSION };
	/*
	 * Parameter block sent by the vendor driver. The meaning of the
	 * individual fields is not documented.
	 */
	static const u8 params[] = {
		ODIN2_GP_CTRL_SET_PARAMS, 0x01,
		0x00, 0x00, 0x00, 0x28,
		0x00, 0x00, 0x00, 0x07,
	};
	int ret;

	ret = odin2_gp_send(gp, ODIN2_GP_CMD_CONTROL, version_req,
			    sizeof(version_req));
	if (ret)
		return ret;

	msleep(100);

	return odin2_gp_send(gp, ODIN2_GP_CMD_CONTROL, params, sizeof(params));
}

static void odin2_gp_grip_work(struct work_struct *work)
{
	struct odin2_gp *gp = container_of(to_delayed_work(work),
					   struct odin2_gp, grip_work);
	DECLARE_BITMAP(values, ARRAY_SIZE(odin2_gp_grip_keymap));
	unsigned int i;
	int ret;

	ret = gpiod_get_array_value_cansleep(gp->grips->ndescs,
					     gp->grips->desc,
					     gp->grips->info, values);
	if (ret) {
		dev_err_ratelimited(&gp->serdev->dev,
				    "failed to read grip buttons: %d\n", ret);
		return;
	}

	guard(mutex)(&gp->report_lock);

	for (i = 0; i < gp->grips->ndescs; i++)
		input_report_key(gp->input, odin2_gp_grip_keymap[i],
				 test_bit(i, values));
	input_sync(gp->input);
}

static irqreturn_t odin2_gp_grip_irq(int irq, void *data)
{
	struct odin2_gp *gp = data;

	mod_delayed_work(system_wq, &gp->grip_work,
			 msecs_to_jiffies(ODIN2_GP_GRIP_DEBOUNCE_MS));

	return IRQ_HANDLED;
}

static void odin2_gp_cancel_grip_work(void *data)
{
	struct odin2_gp *gp = data;

	cancel_delayed_work_sync(&gp->grip_work);
}

static int odin2_gp_setup_grips(struct odin2_gp *gp)
{
	struct device *dev = &gp->serdev->dev;
	unsigned int i;
	int irq, ret;

	gp->grips = devm_gpiod_get_array_optional(dev, "grip", GPIOD_IN);
	if (IS_ERR(gp->grips))
		return dev_err_probe(dev, PTR_ERR(gp->grips),
				     "failed to get grip GPIOs\n");
	if (!gp->grips)
		return 0;

	if (gp->grips->ndescs > ARRAY_SIZE(odin2_gp_grip_keymap))
		return dev_err_probe(dev, -EINVAL, "too many grip GPIOs\n");

	for (i = 0; i < gp->grips->ndescs; i++)
		input_set_capability(gp->input, EV_KEY, odin2_gp_grip_keymap[i]);

	INIT_DELAYED_WORK(&gp->grip_work, odin2_gp_grip_work);

	/* Registered before the IRQs so that it runs after they are freed */
	ret = devm_add_action_or_reset(dev, odin2_gp_cancel_grip_work, gp);
	if (ret)
		return ret;

	for (i = 0; i < gp->grips->ndescs; i++) {
		irq = gpiod_to_irq(gp->grips->desc[i]);
		if (irq < 0)
			return dev_err_probe(dev, irq,
					     "no IRQ for grip GPIO %u\n", i);

		ret = devm_request_irq(dev, irq, odin2_gp_grip_irq,
				       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				       "ayn-odin2-gamepad-grip", gp);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request grip IRQ %d\n", irq);
	}

	return 0;
}

static void odin2_gp_power_off(void *data)
{
	struct odin2_gp *gp = data;

	gpiod_set_value_cansleep(gp->enable_gpio, 0);
}

static int odin2_gp_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct input_dev *input;
	struct odin2_gp *gp;
	int i, ret;

	gp = devm_kzalloc(dev, sizeof(*gp), GFP_KERNEL);
	if (!gp)
		return -ENOMEM;

	gp->serdev = serdev;
	serdev_device_set_drvdata(serdev, gp);

	ret = devm_mutex_init(dev, &gp->report_lock);
	if (ret)
		return ret;

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable vdd supply\n");

	gp->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(gp->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(gp->enable_gpio),
				     "failed to get enable GPIO\n");

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	gp->input = input;
	input->name = "AYN Odin2 Gamepad";
	input->phys = "ayn-odin2-gamepad/input0";
	input->id.bustype = BUS_HOST;

	for (i = 0; i < ARRAY_SIZE(odin2_gp_keymap); i++)
		input_set_capability(input, EV_KEY, odin2_gp_keymap[i]);

	input_set_abs_params(input, ABS_X, -ODIN2_GP_STICK_MAX,
			     ODIN2_GP_STICK_MAX, 0, 0);
	input_set_abs_params(input, ABS_Y, -ODIN2_GP_STICK_MAX,
			     ODIN2_GP_STICK_MAX, 0, 0);
	input_set_abs_params(input, ABS_RX, -ODIN2_GP_STICK_MAX,
			     ODIN2_GP_STICK_MAX, 0, 0);
	input_set_abs_params(input, ABS_RY, -ODIN2_GP_STICK_MAX,
			     ODIN2_GP_STICK_MAX, 0, 0);
	input_set_abs_params(input, ABS_Z, 0, ODIN2_GP_TRIGGER_MAX, 0,
			     ODIN2_GP_TRIGGER_FLAT);
	input_set_abs_params(input, ABS_RZ, 0, ODIN2_GP_TRIGGER_MAX, 0,
			     ODIN2_GP_TRIGGER_FLAT);

	ret = odin2_gp_setup_grips(gp);
	if (ret)
		return ret;

	ret = input_register_device(input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input device\n");

	/* Report the initial grip button state */
	if (gp->grips)
		mod_delayed_work(system_wq, &gp->grip_work, 0);

	serdev_device_set_client_ops(serdev, &odin2_gp_serdev_ops);

	ret = devm_serdev_device_open(dev, serdev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to open UART\n");

	serdev_device_set_baudrate(serdev, ODIN2_GP_BAUDRATE);
	serdev_device_set_flow_control(serdev, false);

	/* Hold the MCU off long enough for it to reset, then start it */
	msleep(100);
	gpiod_set_value_cansleep(gp->enable_gpio, 1);
	ret = devm_add_action_or_reset(dev, odin2_gp_power_off, gp);
	if (ret)
		return ret;

	msleep(100);

	ret = odin2_gp_mcu_init(gp);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialise MCU\n");

	return 0;
}

static const struct of_device_id odin2_gp_of_match[] = {
	{ .compatible = "ayntec,odin2-gamepad" },
	{ }
};
MODULE_DEVICE_TABLE(of, odin2_gp_of_match);

static struct serdev_device_driver odin2_gp_driver = {
	.driver = {
		.name = "ayn-odin2-gamepad",
		.of_match_table = odin2_gp_of_match,
	},
	.probe = odin2_gp_probe,
};
module_serdev_device_driver(odin2_gp_driver);

MODULE_AUTHOR("Teguh Sobirin <teguh@sobir.in>");
MODULE_DESCRIPTION("AYN Odin 2 gamepad MCU driver");
MODULE_LICENSE("GPL");
