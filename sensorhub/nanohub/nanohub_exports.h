#ifndef _NANOHUB_EXPORTS_H_
#define _NANOHUB_EXPORTS_H_

// Exported symbols available to other kernel modules.
//
// Please add the exported symvers to your module's Makefile:
// all: (MAKE) -C $(KERNEL_SRC) M=$(M) modules ...
//      KBUILD_EXTRA_SYMBOLS="$(OUT_DIR)/../exynos-google-cw-extra/drivers/sensorhub/nanohub/Module.symvers"
//
// Your module will have a module dependency on nanohub.

#define MAX_MESSAGE_SIZE 255
#define NANOHUB_AUDIO_CHANNEL_ID 16
#define NANOHUB_SKG_CHANNEL_ID 17

/**
 * Sends a message over a nanohub channel.
 *
 * Remote delivery and dispatch of the entire buffer is guaranteed on successful
 * return.
 *
 * Returns the size of the buffer transferred, or an error if < 0.
 */
extern ssize_t nanohub_send_message(int channel_id, const char *buffer,
                                    size_t length);

#endif /* _NANOHUB_EXPORTS_H_ */
