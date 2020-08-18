/*
 * SNVS Tamper Detection
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc., All Rights Reserved
 */

#ifndef TAMPER_H
#define TAMPER_H

#include "snvsregs.h"
#include "secvio.h"

#define AT5_POLYSEED	0x12345678

/*
 * SNVS Expanded Register
 */
struct snvs_expanded {
	u32 dtocr;
	u32 dtmr;
	u32 dtrr;
	u32 dmcr;
	u32 tpctrl0;
	u32 tpctrl1;
	u32 tpctrl2;
	u32 tpctrl3;
	u32 tpctrl4;
	u32 tpctrl5;
	u32 tpctrl6;
	u32 tpctrl7;
	u32 tpctrl8;
	u32 tpctrl9;
	u32 dslr;
	u32 dhlr;
	u32 smrtn;
};

typedef struct {
	u32 tx;
	u32 rx;
}tamper_active;

typedef struct {
	u32 rx;
	u32 polarity;
}tamper_passive;

typedef struct {
	u32 lpsr;
	u32 lptdsr;
}tamper_status;

/* IOCTL commands */
#define TAMPER_IOCTL_CLOCK_EN		_IOWR('T', 1, bool)
#define TAMPER_IOCTL_TEMP_EN		_IOWR('T', 2, bool)
#define TAMPER_IOCTL_VOLT_EN		_IOWR('T', 3, bool)
#define TAMPER_IOCTL_ACTIVE_EN		_IOWR('T', 4, tamper_active)
#define TAMPER_IOCTL_PASSIVE_EN		_IOWR('T', 5, tamper_passive)
#define TAMPER_IOCTL_GET_STATUS		_IOR('T', 6, tamper_status)
#define TAMPER_IOCTL_CLEAR_STATUS	_IOWR('T', 7, tamper_status)

#endif /* TAMPER_H */
