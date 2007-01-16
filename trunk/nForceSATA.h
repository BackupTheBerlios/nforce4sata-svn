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
 
 
#ifndef _NFORCESATA_
#define _NFORCESATA_

#include <IOKit/IOTypes.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/ata/IOATATypes.h>
#include <IOKit/ata/IOATABusInfo.h>
#include <IOKit/ata/IOPCIATA.h>
#include <IOKit/ppc/IODBDMA.h>
#include <IOKit/IOMemoryCursor.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include "nForceSATAHardware.h"

class nForceSATA : public IOPCIATA
{
    OSDeclareDefaultStructors(nForceSATA)

protected:

	struct ATABusTimings
	{
		UInt8  ataPIOSpeedMode;
		UInt16 ataPIOCycleTime;
		UInt8  ataMultiDMASpeed;
		UInt16 ataMultiCycleTime;
		UInt16 ataUltraDMASpeedMode;
	};
	
	volatile UInt32 *          fATAStatus;
	volatile UInt32 *          fATAError;
	volatile UInt32 *          fATAControl;
	volatile UInt32 *          fInterruptStatus;
	
	UInt32                     fInterruptBitMask;
	UInt32                     fBusChildNum;
	bool                       fIsSata300;
	
	ATABusTimings              fBusTimings[2];
	IOMemoryMap *              fIOBaseAddrMap[5];

	// util functions
	static char* getChipsetName( int chipsetIndex);
	static int   getChipsetIndex( int vendor_id, int device_id );
	
	IOFilterInterruptEventSource * fDeviceInterruptFilter;

public:

	/*--- Overrides from IOService Interface ---*/
	virtual bool init( OSDictionary* properties );
	virtual IOService* probe( IOService* provider,	SInt32*	score );
	
    virtual bool start( IOService* provider );
	virtual void free( void );

    virtual IOWorkLoop * getWorkLoop( void ) const;
    
protected:

	/*--- Overrides from IOATAController Interface ---*/
	/// Mandatory overrides
	// initialize the pointers to the ATA task file registers.
	virtual bool     configureTFPointers( void );
	virtual IOReturn provideBusInfo( IOATABusInfo* infoOut );
	virtual IOReturn getConfig( IOATADevConfig* configRequest, UInt32 unitNumber ); 
	virtual IOReturn selectConfig( IOATADevConfig* configRequest, UInt32 unitNumber );

	/// Optional overrides	
	// timer functions
	virtual void     handleTimeout( void );
	// various protocol phases
	virtual IOReturn synchronousIO(void);

	// hardware access
	virtual IOReturn selectDevice( ataUnitID unit );
	virtual IOReturn issueCommand( void );
	virtual IOReturn writePacket( void );
	virtual IOReturn softResetBus( bool doATAPI = false );

	virtual bool     ATAPISlaveExists( void );
	virtual UInt32   scanForDrives( void );
	virtual IOReturn registerAccess(bool isWrite);

	// device should set the controller to the config for this device
	virtual void selectIOTiming( ataUnitID unit );
	
	/*--- Overrides from IOPCIATA Interface ---*/
	// dma related
	virtual IOReturn startDMA( void );
	virtual IOReturn stopDMA( void );
    virtual bool     allocDMAChannel( void );
    virtual void     initATADMAChains( PRD* descPtr );
    virtual IOReturn createChannelCommands( void );
    virtual bool     freeDMAChannel( void );
	
	// interrupt related
	virtual IOReturn handleDeviceInterrupt(void);

    /* Interrupt event source action/filter */
    static void interruptOccurred( OSObject* owner, IOInterruptEventSource* src, int count );
    static bool interruptFilter( OSObject* owner, IOFilterInterruptEventSource* src );

};

#endif	// _NFORCESATA_
