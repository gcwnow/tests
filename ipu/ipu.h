#ifndef __IPU_H__
#define __IPU_H__

#define JZ4770_CPM_BASE_ADDR	0x10000000
#define JZ4770_IPU_BASE_ADDR	0x13080000

#define BIT(x) (1 << (x))

#define REG_CLKGR0 0x20
#define CLKGR0_IPU BIT(29)

/* Register offset */
#define REG_CTRL		0x0  /* IPU Control Register */
#define REG_STATUS		0x4  /* IPU Status Register */
#define REG_D_FMT		0x8  /* Data Format Register */
#define REG_Y_ADDR		0xc  /* Input Y or YUV422 Packaged Data Address Register */
#define REG_U_ADDR		0x10 /* Input U Data Address Register */
#define REG_V_ADDR		0x14 /* Input V Data Address Register */
#define REG_IN_FM_GS		0x18 /* Input Geometric Size Register */
#define REG_Y_STRIDE		0x1c /* Input Y Data Line Stride Register */
#define REG_UV_STRIDE		0x20 /* Input UV Data Line Stride Register */
#define REG_OUT_ADDR		0x24 /* Output Frame Start Address Register */
#define REG_OUT_GS		0x28 /* Output Geometric Size Register */
#define REG_OUT_STRIDE		0x2c /* Output Data Line Stride Register */
#define REG_RSZ_COEF_INDEX	0x30 /* Resize Coefficients Table Index Register */
#define REG_CSC_CO_COEF		0x34 /* CSC C0 Coefficient Register */
#define REG_CSC_C1_COEF		0x38 /* CSC C1 Coefficient Register */
#define REG_CSC_C2_COEF 	0x3c /* CSC C2 Coefficient Register */
#define REG_CSC_C3_COEF 	0x40 /* CSC C3 Coefficient Register */
#define REG_CSC_C4_COEF 	0x44 /* CSC C4 Coefficient Register */
#define REG_HRSZ_COEF_LUT 	0x48 /* Horizontal Resize Coefficients Look Up Table Register group */
#define REG_VRSZ_COEF_LUT 	0x4c /* Virtical Resize Coefficients Look Up Table Register group */
#define REG_CSC_OFSET_PARA	0x50 /* CSC Offset Parameter Register */
#define REG_Y_PHY_T_ADDR	0x54 /* Input Y Physical Table Address Register */
#define REG_U_PHY_T_ADDR	0x58 /* Input U Physical Table Address Register */
#define REG_V_PHY_T_ADDR	0x5c /* Input V Physical Table Address Register */
#define REG_OUT_PHY_T_ADDR	0x60 /* Output Physical Table Address Register */

// TODO: Stop using jz4770ipu.h altogether.
#define IPU_CTRL_ADDR_SEL	BIT(20)		/* address mode selector */
#define IPU_CTRL_DFIX_SEL	BIT(17)		/* fixed dest addr */
#define IPU_CTRL_LCDC_SEL	BIT(11)		/* output to LCDC FIFO */
#define IPU_CTRL_SPKG_SEL	BIT(10)		/* packed input format */
#define IPU_CTRL_STOP		BIT(7)		/* stop conversion */
#define IPU_CTRL_RST		BIT(6)		/* reset IPU */
#define IPU_CTRL_FM_IRQ_EN	BIT(5)		/* Frame process finish IRQ */
#define IPU_CTRL_VRSZ_EN	BIT(3)		/* vertical resize */
#define IPU_CTRL_HRSZ_EN	BIT(2)		/* horizontal resize */
#define IPU_CTRL_RUN		BIT(1)		/* start conversion */
#define IPU_CTRL_CHIP_EN	BIT(0)		/* chip enable */

#define IPU_STATUS_OUT_END	BIT(0)		/* Frame process finish IRQ */

#define IN_FM_H_SFT	0x0
#define IN_FM_W_SFT	0x10
#define IN_FMT_SFT	0x0
#define OUT_FMT_SFT	0x13

static const char * reg_names[] = {
	"REG_CTRL",
	"REG_STATUS",
	"REG_D_FMT",
	"REG_Y_ADDR",
	"REG_U_ADDR",
	"REG_V_ADDR",
	"REG_IN_FM_GS",
	"REG_Y_STRIDE",
	"REG_UV_STRIDE",
	"REG_OUT_ADDR",
	"REG_OUT_GS",
	"REG_OUT_STRIDE",
	"REG_RSZ_COEF_INDEX",
	"REG_CSC_C0_COEF",
	"REG_CSC_C1_COEF",
	"REG_CSC_C2_COEF",
	"REG_CSC_C3_COEF",
	"REG_CSC_C4_COEF",
	"REG_HRSZ_COEF_LUT",
	"REG_VRSZ_COEF_LUT",
	"REG_CSC_OFFSET",
	"REG_Y_PHY_T_ADDR",
	"REG_U_PHY_T_ADDR",
	"REG_V_PHY_T_ADDR",
	"REG_OUT_PHY_T_ADDR",
};

enum ipu_resize_algorithm {
	IPU_NEAREST_NEIGHBOR,
	IPU_BILINEAR
};

#endif /* __IPU_H__ */
