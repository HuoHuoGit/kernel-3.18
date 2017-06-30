#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>

unsigned char wlan_mac[6];
int valid_wlan_mac = 0;

static int __init wlan_init_mac(char *str)
{
	int i, j;
	short byte1, byte0;
	pr_info("WLAN init with mac address: %s\n", str);
	if (str == NULL) {
		return -1;
	}

	for (i=0, j=0; i < 6; i++,j=j+2) {
		if ((byte1 = hex_to_bin(*(str+j))) < 0)
			return -1;

		if ((byte0 = hex_to_bin(*(str+j+1))) < 0)
			return -1;

		wlan_mac[i] = (unsigned char)(byte1*16+byte0);
	}
	pr_info("WLAN convert to %02x:%02x:%02x:%02x:%02x:%02x\n",
	wlan_mac[0], wlan_mac[1], wlan_mac[2], wlan_mac[3], wlan_mac[4], wlan_mac[5]);
	valid_wlan_mac = 1;
	return 0;
}

unsigned char* kernel_get_wlan_mac(unsigned char *buf)
{
	if (!valid_wlan_mac)
	return NULL;

	memcpy(buf, wlan_mac, 6);
	return buf;

}

early_param("wlan_mac", wlan_init_mac);
EXPORT_SYMBOL(kernel_get_wlan_mac);

