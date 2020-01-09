

int rpmsg_platform_init(void);
void rpmsg_platform_deinit(void);

unsigned int receive_message(void);
int send_message(unsigned int message);


void virtio_set_status(struct virtio_device *vdev, unsigned char status);


