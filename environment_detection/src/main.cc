/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "main_functions.h"
#include "constants.h"

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <math.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/util.h>
#include <stdio.h>

#include <drivers/display.h>
#include <lvgl.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <kernel.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

#include <storage/disk_access.h>
#include <fs/fs.h>
#include <ff.h>


#include <inttypes.h>

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define SECOND 1000

#define MAX_DEVICES 150
#define MAX_PACKETS_RECEIVED 140
#define MAX_DIFFERENT_TX_POWERS 20
#define MAX_DIFFERENT_MAN_PACKET_LEN 20
#define MAX_SERVICES 10

#define SCAN_COUNT 5
#define SCAN_TIME 3 //seconds

#define N_RUNS 10 //about 3 minutes
#define DATA_LINE_LENGTH 46
#define DATA_LENGTH 230

static struct bt_le_scan_param scan_param = {
	.type = BT_HCI_LE_SCAN_ACTIVE,
	.options = BT_LE_SCAN_OPT_NONE,
	.interval = 0x0010,
	.window = 0x0010,
};

static int device_count = 0;
static int beacons_received[MAX_DEVICES][MAX_PACKETS_RECEIVED][2];
static char devices[MAX_DEVICES][BT_ADDR_LE_STR_LEN];
static int old_device_count = 0;
static char old_devices[MAX_DEVICES][BT_ADDR_LE_STR_LEN];

static int txPower[MAX_DIFFERENT_TX_POWERS][2];
static int manufacturer_data_len[MAX_DIFFERENT_MAN_PACKET_LEN][2];

static int different_services = 0;
static int services_count = 0;
static char services[MAX_SERVICES][10];
static bool dev_services[MAX_DEVICES][MAX_SERVICES];

#define KNOWN_SERVICES_LENGTH 23
static char known_services[KNOWN_SERVICES_LENGTH][10] = {
	"0af0", "1802", "180f", "1812", "1826", "2222", "ec88", "fd5a",
	"fd6f", "fdd2", "fddf", "fe03", "fe07", "fe0f", "fe61", "fe9f",
	"fea0", "feb9", "febe", "fee0", "ff0d", "ffc0", "ffe0",
};

static char environments[][50] = {
	"apartment", "house",	    "street",	      "car",	    "train",  "bus",
	"plane",     "supermarket", "clothing_store", "gym",	    "park",   "bar",
	"nature",    "cinema",	    "concert",	      "restaurant", "unknown"

};
const int environment_count = 17;

static bool environment_selected;

static int current_environment;
static char current_env_str[50];
static classification current_classification;

static char daytimes[][50] = { "mo", "no", "ev" };
const int daytime_count = 3;

static bool daytime_selected;

static int current_daytime;
static char current_daytime_str[50];

static int current_data_type;
static char data_types[][20] = { "training", "test" };
const int data_type_count = 2;
static bool data_type_selected;

static int data_sample[230];

static int time_points[4];

const struct device *display_dev;
static lv_obj_t *text;
static bool display_initalized;

static FATFS fat_fs;
/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};
static const char *disk_mount_pt = "/SD:";
const char *dataPath = "/ble_data";
const char *evalPath = "/eval";
static int data_file_count = 0;
static char data_str[3000];
bool sd_card_initialized = false;

const char data_params[] =
	"label, device_count, lost_devices, new_devices, different_services, services_count, txpower_count, tx_power_avg, min_txpower, max_txpower, man_packet_len_count, manufacturer_data_lengths_sum, manufacturer_data_len_avg, avg_received, min_received, max_received, avg_avg_rssi, min_avg_rssi, max_avg_rssi, min_rssi, max_rssi, avg_rssi_difference, avg_avg_difference_between_beacons, avg_difference_first_last";

#define SWA_NODE DT_ALIAS(swa)
#define SWB_NODE DT_ALIAS(swb)
#define SWC_NODE DT_ALIAS(swc)

static const struct gpio_dt_spec buttonA = GPIO_DT_SPEC_GET_OR(SWA_NODE, gpios, { 0 });
static const struct gpio_dt_spec buttonB = GPIO_DT_SPEC_GET_OR(SWB_NODE, gpios, { 0 });
static const struct gpio_dt_spec buttonC = GPIO_DT_SPEC_GET_OR(SWC_NODE, gpios, { 0 });
static struct gpio_callback button_cb_dataA;
static struct gpio_callback button_cb_dataB;
static struct gpio_callback button_cb_dataC;

#define SWA_NODE DT_ALIAS(swa)
#define SWB_NODE DT_ALIAS(swb)
#define SWC_NODE DT_ALIAS(swc)

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

#define LED0 DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN0 DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS0 DT_GPIO_FLAGS(LED0_NODE, gpios)
#define LED1 DT_GPIO_LABEL(LED1_NODE, gpios)
#define PIN1 DT_GPIO_PIN(LED1_NODE, gpios)
#define FLAGS1 DT_GPIO_FLAGS(LED1_NODE, gpios)

const struct device *led0;
const struct device *led1;

int getIndex(char *addr)
{
	for (int i = 0; i < device_count; i++) {
		if (!strcmp(devices[i], addr)) {
			return i;
		}
	}
	return -1;
}

void addDevice(char *addr)
{
	strcpy(devices[device_count], addr);
	if (device_count < MAX_DEVICES) {
		device_count++;
	}
}

void addRssi(int rssi, int index)
{
	for (int i = 0; i < MAX_PACKETS_RECEIVED; i++) {
		if (beacons_received[index][i][0] == 0) {
			beacons_received[index][i][0] = rssi;
			beacons_received[index][i][1] = k_cycle_get_32();
			break;
		}
	}
}

static bool eir_found(struct bt_data *data, void *user_data)
{
	bt_addr_le_t *addr = static_cast<bt_addr_le_t *>(user_data);
	char result[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, result, BT_ADDR_LE_STR_LEN);

	uint8_t txp = 0;
	uint8_t len = 0;

	//printk("[AD]: %u data_len %u\n", data->type, data->data_len);

	switch (data->type) {
	case BT_DATA_TX_POWER:

		txp = data->data[0];

		for (int i = 0; i < MAX_DIFFERENT_TX_POWERS; i++) {
			if (txPower[i][0] == txp) {
				txPower[i][1]++;
				break;
			}
			if (txPower[i][1] == 0) {
				txPower[i][1]++;
				txPower[i][0] = txp;
				break;
			}
		}

		break;
	case BT_DATA_MANUFACTURER_DATA:
		len = data->data_len;
		for (int i = 0; i < MAX_DIFFERENT_MAN_PACKET_LEN; i++) {
			if (manufacturer_data_len[i][0] == len) {
				manufacturer_data_len[i][1]++;
				break;
			}
			if (manufacturer_data_len[i][1] == 0) {
				manufacturer_data_len[i][1]++;
				manufacturer_data_len[i][0] = len;
				break;
			}
		}
		break;

	case BT_DATA_UUID16_SOME:
	case BT_DATA_UUID16_ALL:
		if (data->data_len % sizeof(uint16_t) != 0U) {
			printk("AD malformed\n");
			return true;
		}

		for (int i = 0; i < data->data_len; i += sizeof(uint16_t)) {
			//struct bt_le_conn_param *param;
			struct bt_uuid *uuid;
			uint16_t u16;
			//int err;

			memcpy(&u16, &data->data[i], sizeof(u16));

			struct bt_uuid_16 temp[] = { { .uuid = { BT_UUID_TYPE_16 },
						       .val = (sys_le16_to_cpu(u16)) } };

			uuid = ((struct bt_uuid *)(temp));

			char uuid_str[100];
			bt_uuid_to_str(uuid, uuid_str, sizeof(uuid_str));

			for (int j = 0; j <= different_services; j++) {
				if (!strcmp(services[j], uuid_str)) {
					if (!dev_services[getIndex(result)][j]) {
						dev_services[getIndex(result)][j] = true;
						services_count++;
					}

					return false;
				}
				if (j == different_services) {
					strcpy(services[j], uuid_str);
					different_services++;
					dev_services[getIndex(result)][j] = true;
					services_count++;

					return false;
				}
			}
		}
	}
	return true;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	char result[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, result, BT_ADDR_LE_STR_LEN);

	int index = getIndex(result);

	if (index == -1) {
		//printk("new device found: %s rssi: %d, adv_type: %d \n", result, rssi, adv_type);

		addDevice(result);

		int ind = getIndex(result);
		addRssi(rssi, ind);
	} else {
		addRssi(rssi, index);
	}

	bt_data_parse(buf, eir_found, (void *)addr);
}

void buttonA_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (!data_type_selected) {
		if (current_data_type == 0) {
			current_data_type = 1;
		} else {
			current_data_type = 0;
		}
		printk("set data type to %s\n", data_types[current_data_type]);

	} else if (!environment_selected) {
		if (current_environment < environment_count - 1) {
			current_environment++;
		} else {
			current_environment = 0;
		}

		strcpy(current_env_str, environments[current_environment]);
		printk("set environment to %s\n", current_env_str);

	} else if (!daytime_selected) {
		if (current_daytime < daytime_count - 1) {
			current_daytime++;
		} else {
			current_daytime = 0;
		}

		strcpy(current_daytime_str, daytimes[current_daytime]);
		printk("set daytime to %s\n", current_daytime_str);
	}
}

void buttonB_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (!data_type_selected) {
		data_type_selected = true;
		printk("confirmed data type %s\n", data_types[current_data_type]);

	} else if (!environment_selected) {
		environment_selected = true;
		printk("confirmed environment: %s\n", current_env_str);
	} else if (!daytime_selected) {
		daytime_selected = true;
		printk("confirmed daytime: %s\n", current_daytime_str);
	}
}
void buttonC_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (!data_type_selected) {
		if (current_data_type == 0) {
			current_data_type = 1;
		} else {
			current_data_type = 0;
		}
		printk("set data type to %s\n", data_types[current_data_type]);

	} else if (!environment_selected) {
		if (current_environment > 0) {
			current_environment--;
		} else {
			current_environment = environment_count - 1;
		}

		strcpy(current_env_str, environments[current_environment]);
		printk("set environment to %s\n", current_env_str);
	} else if (!daytime_selected) {
		if (current_daytime > 0) {
			current_daytime--;
		} else {
			current_daytime = daytime_count - 1;
		}

		strcpy(current_daytime_str, daytimes[current_daytime]);
		printk("set daytime to %s\n", current_daytime_str);
	}
}
void initButtons()
{
	// if (!device_is_ready(button.port)) {
	// 	printk("Error: button device %s is not ready\n", button.port->name);
	// 	return;
	// }

	gpio_pin_configure_dt(&buttonA, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&buttonA, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb_dataA, buttonA_pressed, BIT(buttonA.pin));
	gpio_add_callback(buttonA.port, &button_cb_dataA);

	gpio_pin_configure_dt(&buttonB, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&buttonB, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb_dataB, buttonB_pressed, BIT(buttonB.pin));
	gpio_add_callback(buttonB.port, &button_cb_dataB);

	gpio_pin_configure_dt(&buttonC, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&buttonC, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb_dataC, buttonC_pressed, BIT(buttonC.pin));
	gpio_add_callback(buttonC.port, &button_cb_dataC);

	//strcpy(current_env_str, environments[current_environment]);
}

void initLEDs()
{
	led0 = device_get_binding(LED0);
	led1 = device_get_binding(LED1);

	gpio_pin_configure(led0, PIN0, GPIO_OUTPUT_ACTIVE | FLAGS0);
	gpio_pin_configure(led1, PIN1, GPIO_OUTPUT_ACTIVE | FLAGS1);

	gpio_pin_set(led0, PIN0, (int)false);
	gpio_pin_set(led1, PIN1, (int)false);
}

void setLED0(bool on)
{
	gpio_pin_set(led0, PIN0, (int)on);
}
void setLED1(bool on)
{
	gpio_pin_set(led1, PIN1, (int)on);
}

void setDisplayText(char *txt)
{
	if (text != NULL && display_initalized) {
		lv_label_set_text(text, txt);
		lv_obj_align(text, NULL, LV_ALIGN_CENTER, 0, 0);

		lv_task_handler();
	}
}

void initDisplay()
{
	display_dev = device_get_binding(CONFIG_LVGL_DISPLAY_DEV_NAME);

	if (display_dev == NULL) {
		printk("device not found.\n");
		return;
	}

	if (IS_ENABLED(CONFIG_LVGL_POINTER_KSCAN)) {
		lv_obj_t *hello_world_button;

		hello_world_button = lv_btn_create(lv_scr_act(), NULL);
		lv_obj_align(hello_world_button, NULL, LV_ALIGN_CENTER, 0, 0);
		lv_btn_set_fit(hello_world_button, LV_FIT_TIGHT);
		text = lv_label_create(hello_world_button, NULL);
	} else {
		text = lv_label_create(lv_scr_act(), NULL);
	}

	lv_label_set_text(text, "");
	lv_obj_align(text, NULL, LV_ALIGN_CENTER, 0, 0);

	lv_task_handler();
	display_blanking_off(display_dev);

	display_initalized = true;
}

int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		printk("Error opening dir %s [%d]\n", path, res);
		return res;
	}

	int files = 0;
	//printk("\nListing dir %s ...\n", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			//printk("[DIR ] %s\n", entry.name);
		} else {
			files++;
			//printk("[FILE] %s (size = %zu)\n", entry.name, entry.size);
		}
	}

	/* Verify fs_closedir() */
	fs_closedir(&dirp);

	return files;
}

int createDir(const char *path)
{
	struct fs_dir_t dir_data;

	fs_dir_t_init(&dir_data);

	int dir_err = fs_opendir(&dir_data, path);
	if (dir_err) {
		fs_mkdir(path);
		printk("created dir: %s\n", path);
	}
	fs_closedir(&dir_data);

	return lsdir(path);
}

void initSDCard()
{
	/* raw disk i/o */
	do {
		static const char *disk_pdrv = "SD";
		uint64_t memory_size_mb;
		uint32_t block_count;
		uint32_t block_size;

		if (disk_access_init(disk_pdrv) != 0) {
			//LOG_ERR("Storage init ERROR!");
			break;
		}

		if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
			//LOG_ERR("Unable to get sector count");
			break;
		}
		//LOG_INF("Block count %u", block_count);

		if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
			//LOG_ERR("Unable to get sector size");
			break;
		}
		printk("Sector size %u\n", block_size);

		memory_size_mb = (uint64_t)block_count * block_size;
		printk("Memory Size(MB) %u\n", (uint32_t)(memory_size_mb >> 20));
	} while (0);

	mp.mnt_point = disk_mount_pt;

	int res = fs_mount(&mp);

	if (res == FR_OK) {
		printk("Disk mounted.\n");

	} else {
		printk("Error mounting disk.\n");
	}

	for (int j = 0; j < data_type_count; j++) {
		char path[50];
		strcpy(path, disk_mount_pt);
		strcat(path, "/");
		strcat(path, data_types[j]);

		printk("files in %s: %d\n", path, createDir(path));

		char data_path[50];
		strcpy(data_path, path);
		strcat(data_path, dataPath);

		char eval_path[50];
		strcpy(eval_path, path);
		strcat(eval_path, evalPath);

		printk("files in %s: %d\n", data_path, createDir(data_path));
		printk("files in %s: %d\n", eval_path, createDir(eval_path));

		for (int i = 0; i < daytime_count; i++) {
			char dir_day_path[50];
			strcpy(dir_day_path, eval_path);
			strcat(dir_day_path, "/");
			strcat(dir_day_path, daytimes[i]);

			createDir(dir_day_path);

			char dir_day_data_path[50];
			strcpy(dir_day_data_path, data_path);
			strcat(dir_day_data_path, "/");
			strcat(dir_day_data_path, daytimes[i]);

			createDir(dir_day_data_path);
		}
	}

	sd_card_initialized = true;
}

int fileExists(struct fs_file_t *file, char *path)
{
	fs_file_t_init(file);
	int rc;
	rc = fs_open(file, path, FS_O_RDWR);
	if (rc >= 0) {
		fs_close(file);
	}

	return rc;
}

int openOrCreateFile(struct fs_file_t *file, char *path)
{
	fs_file_t_init(file);
	int rc;
	rc = fs_open(file, path, FS_O_CREATE | FS_O_RDWR);
	if (rc < 0 && rc != -2) {
		printk("FAIL: open %s: %d\n", path, rc);
	}

	return rc;
}

void writeDataFile()
{
	struct fs_file_t dataFile;

	int rc;

	char filePath[100];

	//find data file name
	while (1) {
		char first[2];
		sprintf(first, "%c", current_env_str[0]);

		char second[2];
		sprintf(second, "%c", current_env_str[1]);

		char count_str[20];
		sprintf(count_str, "/%s%s%d.csv", first, second, data_file_count);

		char dataFilePath[50];
		strcpy(dataFilePath, disk_mount_pt);
		strcat(dataFilePath, "/");
		strcat(dataFilePath, data_types[current_data_type]);
		strcat(dataFilePath, dataPath);
		strcat(dataFilePath, "/");
		strcat(dataFilePath, current_daytime_str);
		strcat(dataFilePath, count_str);


		rc = fileExists(&dataFile, dataFilePath);

		if (rc == -2) {
			rc = openOrCreateFile(&dataFile, dataFilePath);
			fs_seek(&dataFile, 0, FS_SEEK_SET);
			strcpy(filePath, dataFilePath);
			break;
		}

		data_file_count++;
	}

	fs_write(&dataFile, data_str, strlen(data_str) * sizeof(char));
	fs_sync(&dataFile);

	fs_close(&dataFile);

	static struct fs_dirent entry;
	fs_stat(filePath, &entry);
	printk("created [FILE] %s (size = %zu)\n", entry.name, entry.size);
}

void reset()
{
	for (int i = 0; i < device_count; i++) {
		for (int j = 0; j < MAX_PACKETS_RECEIVED; j++) {
			beacons_received[i][j][0] = 0;
			beacons_received[i][j][1] = 0;
		}
		strcpy(old_devices[i], devices[i]);
		strcpy(devices[i], "");

		for (int j = 0; j < MAX_SERVICES; j++) {
			dev_services[i][j] = false;
		}
	}
	for (int i = 0; i < MAX_SERVICES; i++) {
		strcpy(services[i], "");
	}

	for (int k = 0; k < MAX_DIFFERENT_TX_POWERS; k++) {
		txPower[k][0] = 0;
		txPower[k][1] = 0;
	}

	for (int k = 0; k < MAX_DIFFERENT_MAN_PACKET_LEN; k++) {
		manufacturer_data_len[k][0] = 0;
		manufacturer_data_len[k][1] = 0;
	}
	old_device_count = device_count;
	device_count = 0;
	different_services = 0;
	services_count = 0;
}

void uart_out()
{
	int main(int argc, char *argv[]);
	{
		initDisplay();
		initButtons();
		initLEDs();

		current_data_type = 0;

		current_environment = 0;
		strcpy(current_env_str, environments[current_environment]);

		current_daytime = 0;
		strcpy(current_daytime_str, daytimes[current_daytime]);

		while (!data_type_selected) {
			k_msleep(10);
			setDisplayText(data_types[current_data_type]);
		}

		while (!environment_selected) {
			k_msleep(10);
			setDisplayText(current_env_str);
		}

		while (!daytime_selected) {
			k_msleep(10);
			setDisplayText(current_daytime_str);
		}

		if (!sd_card_initialized) {
			initSDCard();
		}

		setup();

		int err = bt_enable(NULL);
		if (err) {
			printk("Bluetooth init failed (err %d)\n", err);
		}
		printk("Bluetooth initialized\n");

		printk("\nScanning... \n");
		setDisplayText("Scanning...");

		for (int r = 0; r < N_RUNS + 1; r++) {
			// for (int i = 0; i < old_device_count; i++) {
			// 	strcpy(old_devices[i], "");
			// }
			// old_device_count = 0;

			for (int sc = 0; sc < SCAN_COUNT; sc++) {
				time_points[0] = k_cycle_get_32();
				int offset = 0;

				reset();

				for (int i = DATA_LINE_LENGTH * 4 - 1; i >= 0; i--) {
					data_sample[i + DATA_LINE_LENGTH] = data_sample[i];
				}

				err = bt_le_scan_start(&scan_param, scan_cb);
				if (err) {
					printk("Starting scanning failed (err %d)\n", err);
					return;
				}

				k_msleep(SECOND * SCAN_TIME);

				err = bt_le_scan_stop();

				if (err) {
					printk("Stopping scanning failed (err %d)\n", err);
					return;
				}
				time_points[1] = k_cycle_get_32();
				

				printk("\nDevices: %d; services: ", device_count);

				for (int i = 0; i < different_services; i++) {
					printk("%s, ", services[i]);
				}
				printk("\n");

				int new_device_count = 0;
				int lost_device_count = 0;

				for (int k = 0; k < device_count; k++) {
					for (int l = 0; l < old_device_count; l++) {
						if (!strcmp(devices[k], old_devices[l])) {
							break;
						}
						if (l == old_device_count - 1) {
							new_device_count++;
						}
					}
				}

				for (int k = 0; k < old_device_count; k++) {
					for (int l = 0; l < device_count; l++) {
						if (!strcmp(old_devices[k], devices[l])) {
							break;
						}
						if (l == device_count - 1) {
							lost_device_count++;
						}
					}
				}
				data_sample[offset + 0] = device_count;
				data_sample[offset + 1] = lost_device_count;
				data_sample[offset + 2] = new_device_count;

				int txpower_count = 0;
				int txpower_avg = 0;

				int min_txpower = 200;
				int max_txpower = 0;

				for (int k = 0; k < MAX_DIFFERENT_TX_POWERS; k++) {
					if (txPower[k][1] != 0) {
						txpower_count += txPower[k][1];
						txpower_avg += (txPower[k][0] * txPower[k][1]);

						if (txPower[k][0] > max_txpower) {
							max_txpower = txPower[k][0];
						}

						if (txPower[k][0] < min_txpower) {
							min_txpower = txPower[k][0];
						}
					}
				}
				if (txpower_count != 0) {
					txpower_avg /= txpower_count;
				}

				int man_packet_len_count = 0;
				int man_packet_len_avg = 0;
				int man_packet_len_sum = 0;

				for (int k = 0; k < MAX_DIFFERENT_MAN_PACKET_LEN; k++) {
					if (manufacturer_data_len[k][1] != 0) {
						man_packet_len_count += manufacturer_data_len[k][1];
						man_packet_len_avg += (manufacturer_data_len[k][0] *
								       manufacturer_data_len[k][1]);

						man_packet_len_sum += manufacturer_data_len[k][0];
					}
				}
				if (man_packet_len_avg != 0) {
					man_packet_len_avg /= man_packet_len_count;
				}

				data_sample[offset + 3] = different_services;
				data_sample[offset + 4] = services_count;
				data_sample[offset + 5] = txpower_count;
				data_sample[offset + 6] = txpower_avg;
				data_sample[offset + 7] = min_txpower;
				data_sample[offset + 8] = max_txpower;
				data_sample[offset + 9] = man_packet_len_count;
				data_sample[offset + 10] = man_packet_len_sum;
				data_sample[offset + 11] = man_packet_len_avg;

				int avg_received = 0;
				int min_received = MAX_PACKETS_RECEIVED;
				int max_received = 0;

				int avg_avg_rssi = 0;
				int min_rssi = 0;
				int max_rssi = -100;
				int avg_rssi_difference = 0;

				int min_avg_rssi = 0;
				int max_avg_rssi = -100;

				int avg_avg_difference_between_beacons = 0;
				int avg_difference_first_last = 0;

				for (int i = 0; i < device_count; i++) {
					double avg_r = 0;
					int j = 0;

					int current_min_rssi = 0;
					int current_max_rssi = -100;
					int current_avg_bt_bc = 0;
					for (j = 0; j < MAX_PACKETS_RECEIVED; j++) {
						if (beacons_received[i][j][0] == 0) {
							avg_received += j;
							break;
						}

						if (beacons_received[i][j][0] < current_min_rssi) {
							current_min_rssi =
								beacons_received[i][j][0];
						}
						if (beacons_received[i][j][0] > current_max_rssi) {
							current_max_rssi =
								beacons_received[i][j][0];
						}

						avg_r += beacons_received[i][j][0];
						if (j > 0) {
							current_avg_bt_bc +=
								beacons_received[i][j][1] -
								beacons_received[i][j - 1][1];
						}
					}
					if (j < min_received && j != 0) {
						min_received = j;
					}
					if (j > max_received) {
						max_received = j;
					}
					avg_rssi_difference +=
						(current_max_rssi - current_min_rssi);

					if (j != 0) {
						int current_avg = (avg_r / j);
						avg_avg_rssi += current_avg;

						if (current_avg > max_avg_rssi) {
							max_avg_rssi = current_avg;
						}

						if (current_avg < min_avg_rssi) {
							min_avg_rssi = current_avg;
						}
					}
					if (j > 1) {
						avg_avg_difference_between_beacons +=
							(current_avg_bt_bc / (j - 1));
						avg_difference_first_last +=
							beacons_received[i][j - 1][1] -
							beacons_received[i][0][1];
					}

					if (current_min_rssi < min_rssi) {
						min_rssi = current_min_rssi;
					}
					if (current_max_rssi > max_rssi) {
						max_rssi = current_max_rssi;
					}
				}
				if (device_count != 0) {
					avg_received = avg_received / device_count;
					avg_avg_rssi = avg_avg_rssi / device_count;
					avg_rssi_difference = avg_rssi_difference / device_count;
					avg_avg_difference_between_beacons =
						avg_avg_difference_between_beacons / device_count;
					avg_difference_first_last =
						avg_difference_first_last / device_count;
				}

				data_sample[offset + 12] = avg_received;
				data_sample[offset + 13] = min_received;
				data_sample[offset + 14] = max_received;
				data_sample[offset + 15] = avg_avg_rssi;
				data_sample[offset + 16] = min_avg_rssi;
				data_sample[offset + 17] = max_avg_rssi;
				data_sample[offset + 18] = min_rssi;
				data_sample[offset + 19] = max_rssi;
				data_sample[offset + 20] = avg_rssi_difference;
				data_sample[offset + 21] = avg_avg_difference_between_beacons;
				data_sample[offset + 22] = avg_difference_first_last;

				/* services */

				for (int s = 0; s < KNOWN_SERVICES_LENGTH; s++) {
					for (int t = 0; t < different_services; t++) {
						if (!strcmp(known_services[s], services[t])) {
							int s_count = 0;
							for (int u = 0; u < device_count; u++) {
								if (dev_services[u][t]) {
									s_count++;
								}
							}
							data_sample[offset + 23 + s] = s_count;
							break;
						}

						if (t == different_services - 1) {
							data_sample[offset + 23 + s] = 0;
						}
					}
				}
				time_points[2] = k_cycle_get_32();

				if (r != 0) { // to get old devices
					/*     SD-CARD      */

					// printk("\ndata sample: \n[");
					// for (int i = 0; i < 230; i++) {
					// 	printk("%d, ", data_sample[i]);
					// 	k_msleep(10);
					// }
					// printk("]\n");

					loop(&data_sample[0], &current_classification);
					int env_index = current_classification.index;
					int round_prob = (int)round(
						current_classification.probability * 100);

					time_points[3] = k_cycle_get_32();

					printk("true environment: %s (index: %d)\n",
					       current_env_str, current_environment);
					printk("predicted environment: %s (index: %d) (prob: %d%%)\n",
					       available_env[env_index], env_index, round_prob);

					char disp[50];
					strcpy(disp, "t: ");
					strcat(disp, current_env_str);
					strcat(disp, " (");
					strcat(disp, current_daytime_str);
					strcat(disp, ")");
					strcat(disp, "\n\np: ");
					strcat(disp, available_env[env_index]);
					char tmp_1[10];
					sprintf(tmp_1, " %d%%", round_prob);
					strcat(disp, tmp_1);
					setDisplayText(disp);

					if (strcmp(current_env_str, "unknown")) {
						if (!strcmp(current_env_str,
							    available_env[env_index])) {
							setLED0(false);
							setLED1(true);

						} else {
							setLED0(true);
							setLED1(false);
						}
						strcpy(data_str, "");
						strcat(data_str, data_params);
						for (int s = 0; s < KNOWN_SERVICES_LENGTH; s++) {
							strcat(data_str, ", ");
							strcat(data_str, known_services[s]);
						}

						strcat(data_str, ", time_point_1");
						strcat(data_str, ", time_point_2");
						strcat(data_str, ", time_point_3");
						strcat(data_str, "\n");
						strcat(data_str, current_env_str);

						for (int i = 0; i < DATA_LENGTH; i++) {
							if (i % DATA_LINE_LENGTH == 0 && i !=0) {
								for (int j = 1; j < 4; j++) {
									char time_point[20];
									sprintf(time_point, ", %d",
										time_points[j]-time_points[0]);
									strcat(data_str,
									       time_point);
								}
								strcat(data_str, "\n");
								strcat(data_str, current_env_str);
							}
							char value[20];
							sprintf(value, ", %d", data_sample[i]);
							strcat(data_str, value);

						}
						writeDataFile();

						//evaluation

						char env_path[50];
						strcpy(env_path, disk_mount_pt);
						strcat(env_path, "/");
						strcat(env_path, data_types[current_data_type]);
						strcat(env_path, evalPath);
						strcat(env_path, "/");
						strcat(env_path, current_daytime_str);

						strcat(env_path, "/");

						char first[2];
						sprintf(first, "%c", current_env_str[0]);

						char second[2];
						sprintf(second, "%c", current_env_str[1]);

						char last[2];
						sprintf(last, "%c",
							current_env_str[strlen(current_env_str) -
									1]);

						strcat(env_path, first);
						strcat(env_path, second);
						strcat(env_path, "_");
						strcat(env_path, last);
						strcat(env_path, ".txt");

						printk("env file: %s", env_path);

						struct fs_file_t env_file;

						openOrCreateFile(&env_file, env_path);

						char temp[strlen(available_env[env_index]) + 4];
						strcpy(temp, available_env[env_index]);
						strcat(temp, tmp_1);
						strcat(temp, ", ");
						fs_seek(&env_file, 0, FS_SEEK_END);

						fs_write(&env_file, temp,
							 strlen(temp) * sizeof(char));
						fs_sync(&env_file);

						fs_close(&env_file);
					}
				}
			}
		}
		setLED0(true);
		setLED1(true);
		fs_unmount(&mp);
		printk("finished\n");
	}
}

K_THREAD_DEFINE(uart_out_id, STACKSIZE, uart_out, NULL, NULL, NULL, PRIORITY, 0, 0);
