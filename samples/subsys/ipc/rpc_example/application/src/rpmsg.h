

int rpmsg_platform_init(void);
void rpmsg_platform_deinit(void);

unsigned int receive_message(u8_t * buff);
int send_message(u8_t * buff, u8_t len);


void virtio_set_status(struct virtio_device *vdev, unsigned char status);


void printf_buff(u8_t * buff, u8_t size);
