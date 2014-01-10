#include "ipu.h"
#include "ratios.h"

#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

struct ipu {
	void *base;
	unsigned long fb;
	int dev_mem_fd;
};

static struct ipu *ipu;

static uint32_t read_reg(struct ipu *ipu, unsigned int reg)
{
	return *(uint32_t *) (ipu->base + reg);
}

static void write_reg(struct ipu *ipu, unsigned int reg, uint32_t value)
{
	*(uint32_t *) (ipu->base + reg) = value;
}

#define set_bit(ipu, reg, bit) \
  write_reg(ipu, reg, read_reg(ipu, reg) | (bit))

#define clr_bit(ipu, reg, bit) \
  write_reg(ipu, reg, read_reg(ipu, reg) & ~(bit))

static void print_regs(struct ipu *ipu)
{
	unsigned int i;
	for (i = 0; i < sizeof(reg_names) / sizeof(reg_names[0]); i++) {
		printf("%s = 0x%08x\n", reg_names[i], read_reg(ipu, i * 4));
	}
}

static void ipu_wait_completion(struct ipu *ipu)
{
	while (!(read_reg(ipu, REG_STATUS) & IPU_STATUS_OUT_END))
		usleep(4000);
}

static void ipu_stop(struct ipu *ipu, int force)
{
	if (!force) {
		uint32_t ctrl = read_reg(ipu, REG_CTRL);
		if (ctrl & IPU_CTRL_CHIP_EN) {
			set_bit(ipu, REG_CTRL, IPU_CTRL_STOP);
			ipu_wait_completion(ipu);
		}
	}

	write_reg(ipu, REG_CTRL, 0);
	write_reg(ipu, REG_STATUS, 0);
	set_bit(ipu, REG_CTRL, IPU_CTRL_RST);
}

static const struct mn *find_mn(float ratio)
{
	unsigned int i, idx = 0;
	float min = 100.0f;

	for (i = 0; i < sizeof(ipu_ratio_table) / sizeof(struct mn); i++) {
		float diff = ipu_ratio_table[i].ratio - ratio;
		if (diff < 0)
			diff = -diff;
		if (diff < min) {
			min = diff;
			idx = i;
		}
	}

	printf("N/M found for ratio %f: %u/%u, idx=%u\n",
				ratio, ipu_ratio_table[idx].n, ipu_ratio_table[idx].m, idx);
	return &ipu_ratio_table[idx];
}

static unsigned int calc_size(unsigned int src, const struct mn *mn)
{
	unsigned int size = (float) ((src - 1) * mn->m) / (float) mn->n;
	float tmp = (float) (size * mn->n) / (float) mn->m;
	return size + (tmp != (float) (src - 1)) + (mn->m >= mn->n);
}

/* XXX: This does not work. */
static void ipu_set_resize_coef(struct ipu *ipu,
			const struct mn *mn, unsigned int reg)
{
	float ratio = (float) mn->m / (float) mn->n;
	unsigned int coef_raw[32];
    const float fixpoint_lut_coef = 512.0;
	unsigned int i, t, value;

	write_reg(ipu, reg, 1);
	coef_raw[0] = fixpoint_lut_coef;

	for (i = 1, t = 1; i < mn->m; i++) {
		float factor = (float) i * ratio;
		factor -= (int) factor;
		float w_coef = 1.0 - factor;
		coef_raw[i] = (unsigned int) (fixpoint_lut_coef * w_coef);

		value = (coef_raw[i - 1] << 6) | ((t++ <= factor) << 1);
		printf("Writing value idx %i: %u\n", i - 1, value);
		write_reg(ipu, reg, value);
	}

	value = (coef_raw[i - 1] << 6) | 2;
	write_reg(ipu, reg, value);
	printf("Writing value idx %i: %u\n", i - 1, value);
}

static void ipu_set_resize_params(struct ipu *ipu,
			unsigned int srcW, unsigned int srcH,
			unsigned int dstW, unsigned int dstH,
			unsigned int bytes_per_pixel)
{
	int upscaleW = (dstW >= srcW), upscaleH = (dstH >= srcH);
	const struct mn *mnW, *mnH;

	write_reg(ipu, REG_RSZ_COEF_INDEX, (31 << 16) | 31);

	if (srcW == dstW) {
		mnW = &ipu_ratio_table[0];
	} else {
		float ratio = (float) (dstW - 1 - upscaleW) / (float) (srcW - 1);
		mnW = find_mn(ratio);
	}

	if (srcH == dstH) {
		mnH = &ipu_ratio_table[0];
	} else {
		float ratio = (float) (dstH - 1 - upscaleH) / (float) (srcH - 1);
		mnH = find_mn(ratio);
	}

	/* Set the resize coefficients */
	ipu_set_resize_coef(ipu, mnW, REG_HRSZ_COEF_LUT);
	ipu_set_resize_coef(ipu, mnH, REG_VRSZ_COEF_LUT);

	/* Calculate valid W/H parameters */
	dstW = calc_size(srcW, mnW);
	dstH = calc_size(srcH, mnH);
	printf("New output size: %ux%u\n", dstW, dstH);

	/* Set the input/output height/width */
	write_reg(ipu, REG_IN_FM_GS,
				((srcW * bytes_per_pixel) << IN_FM_W_SFT)
				| (srcH << IN_FM_H_SFT));
	write_reg(ipu, REG_OUT_GS,
				((dstW * bytes_per_pixel) << IN_FM_W_SFT)
				| (dstH << IN_FM_H_SFT));

	/* Set the input/output stride */
	write_reg(ipu, REG_Y_STRIDE, srcW * bytes_per_pixel);
	write_reg(ipu, REG_OUT_STRIDE, dstW * bytes_per_pixel);
}

static void ipu_reset(struct ipu *ipu,
			unsigned int srcW, unsigned int srcH,
			unsigned int dstW, unsigned int dstH)
{
	ipu_stop(ipu, 0);

	/* Enable the chip and packed mode */
	write_reg(ipu, REG_CTRL, IPU_CTRL_CHIP_EN | IPU_CTRL_SPKG_SEL);

	/* Set input/output pixel format to rgb888 */
	write_reg(ipu, REG_D_FMT, (2 << OUT_FMT_SFT) | (2 << IN_FMT_SFT));

	/* Set the input/output addresses */
	write_reg(ipu, REG_Y_ADDR, (uint32_t) ipu->fb);
	write_reg(ipu, REG_OUT_ADDR, ipu->fb + 320 * 240 * 4);

	printf("Setting the resize params...\n");
	/* Set the resize params */
	ipu_set_resize_params(ipu, srcW, srcH, dstW, dstH, 4);
}

static void ipu_run(struct ipu *ipu)
{
	set_bit(ipu, REG_CTRL, IPU_CTRL_RUN);
}

static void ipu_control_clock(struct ipu *ipu, int enable)
{
	if (enable)
		ipu_stop(ipu, 1);

	void *addr = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
				MAP_SHARED, ipu->dev_mem_fd, JZ4770_CPM_BASE_ADDR);
	if (!addr) {
		fprintf(stderr, "Unable to mmap register clkgr0\n");
	} else {
		uint32_t *ptr = addr + REG_CLKGR0;
		if (enable)
			*ptr &= ~CLKGR0_IPU;
		else
			*ptr |= CLKGR0_IPU;
		munmap(ptr, 0x4);
	}
}

#define ipu_enable_clock(ipu) ipu_control_clock(ipu, 1)
#define ipu_disable_clock(ipu) ipu_control_clock(ipu, 0)

static void ipu_run_test(struct ipu *ipu)
{
	printf("Enabling clock...\n");
	ipu_enable_clock(ipu);
	printf("Clock enabled. Reseting IPU...\n");
	ipu_reset(ipu, 320, 240, 320, 120);
	printf("IPU reseted. Running IPU...\n");
	ipu_run(ipu);
	printf("IPU should be running now. Waiting for EOF status bit...\n");
	ipu_wait_completion(ipu);
	printf("Sequence completed! IPU exited with status code %x.\n\n",
				read_reg(ipu, REG_STATUS) & 0x7);
	print_regs(ipu);
	printf("\nStopping clock...\n");
	ipu_disable_clock(ipu);
}

static void quit_all(int err)
{
	if (ipu) {
		ipu_disable_clock(ipu);
		close(ipu->dev_mem_fd);
		munmap(ipu->base, 0x64);
		free(ipu);
	}

	exit(err);
}

static void set_handler(int signal, void (*handler)(int))
{
	struct sigaction sig;
	sigaction(signal, NULL, &sig);
	sig.sa_handler = handler;
	sigaction(signal, &sig, NULL);
}

int main(void)
{
	set_handler(SIGINT, &quit_all);
	set_handler(SIGSEGV, &quit_all);
	set_handler(SIGTERM, &quit_all);

	int fd = open("/dev/fb0", O_RDONLY);
	if (fd < 0) {
		fprintf(stderr, "Unable to open framebuffer\n");
		return EXIT_FAILURE;
	}

	struct  fb_fix_screeninfo fbinfo;
	if (ioctl(fd, FBIOGET_FSCREENINFO, &fbinfo) < 0) {
		fprintf(stderr, "get fixed screen info failed\n");
		close(fd);
		return EXIT_FAILURE;
	}

	close(fd);

	fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Unable to open /dev/mem\n");
		return EXIT_FAILURE;
	}

	ipu = malloc(sizeof(*ipu));
	if (!ipu) {
		fprintf(stderr, "Unable to allocate memory\n");
		close(fd);
		return EXIT_FAILURE;
	}

	void *addr = mmap(NULL, 0x64, PROT_READ | PROT_WRITE,
				MAP_SHARED, fd, JZ4770_IPU_BASE_ADDR);
	if (!addr) {
		fprintf(stderr, "Unable to mmap /dev/mem\n");
		free(ipu);
		close(fd);
		return EXIT_FAILURE;
	}

	ipu->base = addr;
	ipu->fb = fbinfo.smem_start;
	ipu->dev_mem_fd = fd;

	printf("Framebuffer physical address: 0x%lx\n", fbinfo.smem_start);

	ipu_run_test(ipu);
	quit_all(EXIT_SUCCESS);

	return 0;
}
