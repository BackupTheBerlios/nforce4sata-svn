/*
 * Copyright (c) 2002-2004 medevil <medevil84@gmail.com> Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * The contents of this file constitute Original Code as defined in and
 * are subject to the Apple Public Source License Version 1.1 (the
 * "License").  You may not use this file except in compliance with the
 * License.  Please obtain a copy of the License at
 * http://www.apple.com/publicsource and read it before using this file.
 * 
 * This Original Code and all software distributed under the License are
 * distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT.  Please see the
 * License for the specific language governing rights and limitations
 * under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 */

#ifndef _NFORCESATAHARDWARE_
#define _NFORCESATAHARDWARE_ 

#if     1
#define DEBUG_LOG(fmt, args...) IOLog(fmt, ## args)
#define ERROR_LOG(fmt, args...) IOLog(fmt, ## args)
#else
#define DEBUG_LOG(fmt, args...) 
#define ERROR_LOG(fmt, args...) IOLog(fmt, ## args)
#endif


// Bit-significant representation of supported modes. 
#define kATASupportedPIOModes		 0x1F	// modes 4, 3, 2, 1, and 0
#define kATASupportedMultiDMAModes	 0x07	// modes 2, 1, and 0
#define	kATASupportedUltraDMAModes	 0x3F	// modes 5, 4, 3, 2, 1, and 0

#define kATASecondaryCodecOffset     0x40

// Increase the PRD table size to one full page or 4096 descriptors for
// large transfers via DMA.  2048 are required for 1 megabyte transfers
// assuming no fragmentation and no alignment issues on the buffer.  We
// allocate twice that since there are more issues than simple alignment
// for this DMA engine.

#define kATAXferDMADesc  512
#define kATAMaxDMADesc   kATAXferDMADesc

// up to 2048 ATA sectors per transfer
#define kMaxATAXfer      512 * 2048

// max two nubs per channel
#define kMaxDriveCount   2

#define ATA_NVIDIA_ID           0x10de
#define ATA_NFORCE2_PRO_S1      0x008e10de
#define ATA_NFORCE3_PRO_S1      0x00e310de
#define ATA_NFORCE3_PRO_S2      0x00ee10de
#define ATA_NFORCE_MCP04_S1     0x003610de
#define ATA_NFORCE_MCP04_S2     0x003e10de
#define ATA_NFORCE_CK804_S1     0x005410de
#define ATA_NFORCE_CK804_S2     0x005510de
#define ATA_NFORCE_MCP51_S1     0x026610de
#define ATA_NFORCE_MCP51_S2     0x026710de
#define ATA_NFORCE_MCP55_S1     0x037e10de
#define ATA_NFORCE_MCP55_S2     0x037f10de

#define NV4             0x0010
#define NVQ             0x0020
#define ATA_SA150       0x47
#define ATA_SA300       0x48

// This is the list of the supported nForce Chipsets.
typedef struct nForceSATAChipset {
	UInt32	chipid;
	UInt32	flags;
	UInt8	max_dma;
	char	*text;
} nForceSATAChipset;

static nForceSATAChipset nforceChipsets[] = {
	{ ATA_NFORCE2_PRO_S1,  0,       ATA_SA150, "nForce2 Pro" },
	{ ATA_NFORCE3_PRO_S1,  0,       ATA_SA150, "nForce3 Pro" },
	{ ATA_NFORCE3_PRO_S2,  0,       ATA_SA150, "nForce3 Pro" }
	{ ATA_NFORCE_MCP04_S1, NV4,     ATA_SA150, "nForce MCP" },
	{ ATA_NFORCE_MCP04_S2, NV4,     ATA_SA150, "nForce MCP" },
	{ ATA_NFORCE_CK804_S1, NV4,     ATA_SA300, "nForce CK804" },
	{ ATA_NFORCE_CK804_S2, NV4,     ATA_SA300, "nForce CK804" },
	{ ATA_NFORCE_MCP51_S1, NV4|NVQ, ATA_SA300, "nForce MCP51" },
	{ ATA_NFORCE_MCP51_S2, NV4|NVQ, ATA_SA300, "nForce MCP51" },
	{ ATA_NFORCE_MCP55_S1, NV4|NVQ, ATA_SA300, "nForce MCP55" },
	{ ATA_NFORCE_MCP55_S2, NV4|NVQ, ATA_SA300, "nForce MCP55" },
	{ 0, 0, 0, 0, 0, "nForce Unknown"}
};

UInt8 timings[] = { 
	0xa8, 0x65, 0x42, 0x22, 0x20, 0x42, 0x22, 
	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20 
};

// nForce as similiar via timing modes if ata is <= ATA133.
int modes[][7] = {
	{ 0xc2, 0xc1, 0xc0, 0x00, 0x00, 0x00, 0x00 },	/* VIA ATA33 */
	{ 0xee, 0xec, 0xea, 0xe9, 0xe8, 0x00, 0x00 },	/* VIA ATA66 */
	{ 0xf7, 0xf6, 0xf4, 0xf2, 0xf1, 0xf0, 0x00 },	/* VIA ATA100 */
	{ 0xf7, 0xf7, 0xf6, 0xf4, 0xf2, 0xf1, 0xf0 },	/* VIA ATA133 */
	{ 0xc2, 0xc1, 0xc0, 0xc4, 0xc5, 0xc6, 0xc7 }	/* AMD/nVIDIA */
};

#endif // _NFORCESATAHARDWARE_
