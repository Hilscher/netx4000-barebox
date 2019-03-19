
#define FW_REQUEST  1
#define FW_RELEASE  2
#define FW_UPLOAD   3
#define FW_RESET    4
#define FW_START    5
#define FW_STOP     6

struct ioctl_request {
	struct device_d *dev;
	char *id;
	void **data;
};
