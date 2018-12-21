/*
 * serdes-benchmark.c
 *
 * Magic Leap SERDES benchmark driver
 *
 * Copyright (c) 2017, Magic Leap, Inc.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/ml-mux-client.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#define SB_PATTERN_COUNT (64)

#define SB_PAYLOAD_LENGTH (1016)

static char expected_test_patterns[SB_PATTERN_COUNT][SB_PAYLOAD_LENGTH];

/*
 * Device structure that contains info related to SERDES benchmarking
 * channel.
 */
struct serdes_benchmark_dev {
	struct device *dev;
	struct ml_mux_client client;
	const char *chan_name;
};

/*
 * Callback function that is executed when the benchmark channel
 * has been created on the wearable.
 */
static void serdes_mux_benchmark_open(struct ml_mux_client *cli, bool is_open)
{
	struct serdes_benchmark_dev *sb_device;

	sb_device = container_of(cli, struct serdes_benchmark_dev, client);
}

/*
 * Callback function that handles incoming packets from the wearable.
 */
static void sb_mux_recv_cb(struct ml_mux_client *cli, uint32_t len, void *msg)
{
	static int error;
	static unsigned int number_of_correct_packets;
	static unsigned char index;
	struct serdes_benchmark_dev *sb_device;

	sb_device = container_of(cli, struct serdes_benchmark_dev, client);

	/*
	 * Once an error occurs, stop the test.
	 */
	if (error)
		return;

	/*
	 * Test the actual received packet against the
	 * the expected packet.
	 */
	if (len == SB_PAYLOAD_LENGTH) {
		char *pattern = expected_test_patterns[index];

		if (memcmp(msg, pattern, SB_PAYLOAD_LENGTH) == 0)
			++number_of_correct_packets;
	} else
		error = 1;

	/*
	 * Switch to next expected packet.
	 */
	++index;
	index %= SB_PATTERN_COUNT;

	/*
	 * Report error once.
	 */
	if (error) {
		dev_info(sb_device->dev, "sb_mux_recv_cb: error occurred\n");
		dev_info(sb_device->dev, "sb_mux_recv_cb: number_of_correct_packets = %d\n",
					number_of_correct_packets);

		ml_mux_send_msg(cli->ch, sizeof(number_of_correct_packets),
						&number_of_correct_packets);
	}
}
/*
 * Ascertain the multiplexer channel that will be used to benchmark
 * the SERDES link.
 */
static int serdes_mux_benchmark_parse_dt(struct serdes_benchmark_dev *sb_device)
{
	struct device_node *np = sb_device->dev->of_node;

	if (!np)
		return -ENODEV;

	if (of_property_read_string(np, "ml,chan-name",
		 &sb_device->chan_name)) {
		dev_err(sb_device->dev, "ml,chan-name undefined!\n");
		return -EINVAL;
	}

	return 0;
}
/*
 * Initializes SERDES benchmark driver by opening the "benchmark"
 * multiplexer channel.
 */
static int serdes_mux_benchmark_probe(struct platform_device *pdev)
{
	int rc;
	struct serdes_benchmark_dev *sb_device;
	unsigned char i;

	sb_device = devm_kzalloc(&pdev->dev,
					sizeof(struct serdes_benchmark_dev),
					GFP_KERNEL);

	if (!sb_device)
		return -ENOMEM;

	sb_device->dev = &pdev->dev;
	platform_set_drvdata(pdev, sb_device);

	sb_device->client.dev = sb_device->dev;
	sb_device->client.notify_open = serdes_mux_benchmark_open;
	sb_device->client.receive_cb = sb_mux_recv_cb;

	if (serdes_mux_benchmark_parse_dt(sb_device))
		return -EINVAL;

	rc = ml_mux_request_channel(&sb_device->client,
		(char *)sb_device->chan_name);

	if (rc < 0)
		return rc;

	for (i = 0; i < SB_PATTERN_COUNT; i++)
		memset(expected_test_patterns[i], i, SB_PAYLOAD_LENGTH);

	return 0;
}
/*
 * Function called when benchmark driver is unloaded.
 */
static int serdes_mux_benchmark_remove(struct platform_device *pdev)
{
	struct serdes_benchmark_dev *sb_device = platform_get_drvdata(pdev);

	ml_mux_release_channel(sb_device->client.ch);

	return 0;
}

static const struct of_device_id serdes_mux_benchmark_dt_match[] = {
	{ .compatible = "ml,serdes-benchmark" },
	{}
};
MODULE_DEVICE_TABLE(of, serdes_mux_benchmark_dt_match);

static struct platform_driver serdes_mux_benchmark_driver = {
	.driver = {
		.name = "serdes-benchmark",
		.of_match_table = serdes_mux_benchmark_dt_match,
	},
	.probe  = serdes_mux_benchmark_probe,
	.remove = serdes_mux_benchmark_remove,
};

module_platform_driver(serdes_mux_benchmark_driver);

MODULE_AUTHOR("Magic Leap, Inc.");
MODULE_DESCRIPTION("Magic Leap SERDES benchmark driver");
MODULE_LICENSE("GPL v2");
