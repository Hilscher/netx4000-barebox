#ifndef __NETX4000_XC_HAL_H
#define __NETX4000_XC_HAL_H

#ifdef __BAREBOX_CODE
	/* Barebox specific code */
#else
	/* Linux specific code */
#endif

#define _HW_CONCAT(a,b) a ## b

#define HW_MSK(bf)           _HW_CONCAT(MSK_NX4000_, bf)
#define HW_DFLT_VAL(reg)     _HW_CONCAT(DFLT_VAL_NX4000_, reg)

typedef enum {
	XC_TYPE_RPEC = 0,
	XC_TYPE_TPEC,
	XC_TYPE_RPU,
	XC_TYPE_TPU
} XC_TYPE_E;

struct xc_res {
	uint8_t xcinst;
	uint8_t xcNo;
	uint8_t xcPortNo;

	struct resource *rpec_res;
	struct resource *tpec_res;
	struct resource *rpu_res;
	struct resource *tpu_res;

	struct resource *rpec_reg_res;
	struct resource *tpec_reg_res;
	struct resource *xmac_reg_res;

	struct resource *irq_reg_res;

	struct resource *startstop_res;

	void* __iomem rpec_pram_area;
	void* __iomem tpec_pram_area;
	void* __iomem rpu_pram_area;
	void* __iomem tpu_pram_area;

	NX4000_XPEC_AREA_T* __iomem rpec_reg_area;
	NX4000_XPEC_AREA_T* __iomem tpec_reg_area;
	NX4000_XMAC_AREA_T* __iomem xmac_reg_area;

	uint32_t* __iomem irq_reg_area;

	/* shared iomem */
	NX4000_XC_START_STOP_AREA_T* __iomem startstop_area;
};

int xc_release_xc_res(struct xc_res *xcRes);
struct xc_res *xc_alloc_xc_res(struct device_d *dev, uint32_t uiPort);
int xc_reset(struct xc_res* xc);
int xc_load( struct xc_res* x, XC_TYPE_E eXcType, const uint32_t* pulXcPrg);
int xc_start(struct xc_res* xc);
int xc_stop(struct xc_res* xc);

#endif



