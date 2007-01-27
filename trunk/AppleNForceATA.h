/*
 * Copyright (c) 2004 Apple Computer, Inc. All rights reserved.
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

#ifndef _APPLENFORCEATA_H
#define _APPLENFORCEATA_H

#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/ata/IOATATypes.h>
#include <IOKit/ata/IOPCIATA.h>
#include <IOKit/ata/IOATAController.h>
#include <IOKit/ata/ATADeviceNub.h>
#include "AppleNForceATAHardware.h"
#include "AppleNForceATAChannel.h"
#include "AppleNForceATATiming.h"

struct ATABusTimings
{
    UInt8                      pioModeNumber;
    const TimingParameter *    pioTiming;
    UInt8                      dmaModeNumber;
    const TimingParameter *    dmaTiming;
    UInt8                      ultraModeNumber;
    bool                       ultraEnabled;
};

class AppleNForceATA : public IOPCIATA
{
    OSDeclareDefaultStructors( AppleNForceATA )

protected:
	 
    /*
	 * References to parent(s), and other objects.
	 */
    AppleNForceATAChannel *        fProvider;
	IOInterruptEventSource *       fInterruptSource;
    IOWorkLoop *                   fWorkLoop;
	
    ATABusTimings                  fBusTimings[ kMaxDriveCount ];
    bool                           f80PinCablePresent;
    UInt16                         fBMBaseAddr;
    UInt32                         fChannelNumber;
    UInt32                         fUltraModeMask;
	IOATARegPtr32                  fInterryptStatus;
    bool                           fHardwareLostContext;

    /* Interrupt functions */

    static void      interruptOccurred( OSObject *               owner,
                                        IOInterruptEventSource * src,
                                        int                      count );
    static bool      interruptFilter( OSObject * owner,
                                      IOFilterInterruptEventSource * src );

	virtual IOReturn handleDeviceInterrupt( void );
	bool             interruptIsValid( IOFilterInterruptEventSource * src );

    /* Driver functions */
	
    virtual bool     getBMBaseAddress( UInt32   channel,
                                       UInt16 * baseAddr );

    virtual void     resetBusTimings( void );

    virtual void     selectIOTiming( ataUnitID unit );

    virtual bool     setDriveProperty( UInt32       driveUnit,
                                       const char * key,
                                       UInt32       value,
                                       UInt32       numberOfBits);

    virtual IOReturn synchronousIO( void );

    virtual void     initForPM( IOService * provider );

    virtual void     selectTimingParameter( IOATADevConfig * configRequest,
                                            UInt32           unitNumber );

    virtual void     programTimingRegisters( void );

    virtual void     writeTimingIntervalNS( TimingReg reg,
                                            UInt32       unit,
                                            UInt32       timeNS );

    virtual void     writeTimingRegister( TimingReg reg,
                                          UInt32       unit,
                                          UInt8        periods );

    virtual UInt32   readTimingIntervalNS( TimingReg reg, UInt32 unit );

    virtual UInt8    readTimingRegister( TimingReg reg, UInt32 unit );

    virtual void     initializeHardware( void );

    virtual void     dumpHardwareRegisters( void );

public:
    /* IOService overrides */

    virtual bool     start( IOService * provider );

    virtual void     stop( IOService * provider );

    virtual void     free( void );

    virtual IOWorkLoop * getWorkLoop( void ) const;

    virtual IOReturn message( UInt32      type,
                              IOService * provider,
                              void *      argument );

    virtual IOReturn setPowerState( unsigned long stateIndex,
                                    IOService *   whatDevice );

    /* Mandatory IOATAController overrides */

    virtual bool     configureTFPointers( void );

    virtual IOReturn provideBusInfo( IOATABusInfo * infoOut );

    virtual IOReturn getConfig( IOATADevConfig * configOut,
                                UInt32           unit );

    virtual IOReturn selectConfig( IOATADevConfig * config,
                                   UInt32           unit );

    /* Optional IOATAController overrides */

    virtual UInt32   scanForDrives( void );

    virtual IOReturn handleQueueFlush( void );

    /* Optional IOPCIATA overrides to support large transfers */

    virtual bool     allocDMAChannel( void );

    virtual void     initATADMAChains( PRD * descPtr );

    virtual IOReturn createChannelCommands( void );

    virtual bool     freeDMAChannel( void );
	
	/* Debug info overrides (can be removed without problems!) */
	virtual IOReturn writePacket( void );
	//virtual IOReturn registerAccess(bool isWrite);
	virtual IOReturn selectDevice( ataUnitID unit );
	//virtual bool waitForU8Status (UInt8 mask, UInt8 value);
	virtual IOReturn asyncIO(void);

	// override from IOATAController
	// activate the DMA engine as per the current command
	virtual IOReturn startDMA( void );

	// override from IOATAController
	// safely halt the DMA engine  regardless of state
	virtual IOReturn stopDMA( void );
	
	virtual bool busCanDispatch( void );
	virtual IOReturn dispatchNext( void );
	virtual IOReturn handleExecIO( void );
	virtual IOReturn handleBusReset( void );
	virtual IOReturn softResetBus( bool doATAPI = false );
	
};

#endif /* !_APPLENFORCEATA_H */
