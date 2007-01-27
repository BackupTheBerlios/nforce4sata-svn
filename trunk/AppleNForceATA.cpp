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

#include <sys/systm.h>    // snprintf
#include <IOKit/assert.h>
#include <IOKit/IOMessage.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/storage/IOStorageProtocolCharacteristics.h>
#include "AppleNForceATA.h"

// for wait U8Status, loop time in uS
#define kStatusDelayTime  5
// how many times through the loop for a MS.
#define kStatusDelayLoopMS  1000 / kStatusDelayTime

#define super IOPCIATA
OSDefineMetaClassAndStructors( AppleNForceATA, IOPCIATA )

#define kPIOModeMask   ((1 << kPIOModeCount) - 1)
#define kDMAModeMask   ((1 << kDMAModeCount) - 1)
#define kUDMAModeMask  0x7F
//(fProvider->getUltraDMAModeMask())

#define DRIVE_IS_PRESENT(u) \
        (_devInfo[u].type != kUnknownATADeviceType)

#define TIMING_PARAM_IS_VALID(p) \
        ((p) != 0)

// Increase the PRD table size to one full page or 4096 descriptors for
// large transfers via DMA.  2048 are required for 1 megabyte transfers
// assuming no fragmentation and no alignment issues on the buffer.  We
// allocate twice that since there are more issues than simple alignment
// for this DMA engine.

#define kATAXferDMADesc  512
#define kATAMaxDMADesc   kATAXferDMADesc

// up to 2048 ATA sectors per transfer

#define kMaxATAXfer      512 * 2048

/*---------------------------------------------------------------------------
 *
 * Start the single-channel NVIDIA ATA controller driver.
 *
 ---------------------------------------------------------------------------*/

bool AppleNForceATA::start( IOService * provider )
{
    bool superStarted = false;
    UInt8	cableDetect;
    UInt32	udmaTiming;

    DEBUG_LOG("%s: %s( %p, %p )\n", getName(), __FUNCTION__, this, provider);

    // Our provider is a 'nub' that represents a single channel PCI ATA
    // controller, and not an IOPCIDevice.

    fProvider = OSDynamicCast( AppleNForceATAChannel, provider );
    if ( fProvider == 0 )
        goto fail;

    // Retain and open our provider.

    fProvider->retain();

    if ( fProvider->open( this ) != true )
    {
        DEBUG_LOG("%s: provider open failed\n", getName());
        goto fail;
    }

    // Create a work loop.

    fWorkLoop = IOWorkLoop::workLoop();
    if ( fWorkLoop == 0 )
    {
        DEBUG_LOG("%s: new work loop failed\n", getName());
        goto fail;
    }

    // Cache static controller properties.

    fChannelNumber = fProvider->getChannelNumber();
    if ( fChannelNumber > SEC_CHANNEL_ID )
    {
        DEBUG_LOG("%s: bad ATA channel number %ld\n", getName(),
                  fChannelNumber);
        goto fail;
    }
	
	fUltraModeMask = (1 << (PCI_HW_UDMA_133 + 1)) - 1;

    // Probe for 80-pin conductors on drive 0 and 1.

    cableDetect = fProvider->pciConfigRead8(PCI_CABLE_DETECT);
    udmaTiming  = fProvider->pciConfigRead32(PCI_ULTRA_TIMING);

    if (cableDetect & (0x3 << (fChannelNumber * 2)))
        f80PinCablePresent = true;

    // If system booted with UDMA modes that require a 80-pin cable,
    // then BIOS must know something we don't.

    if (udmaTiming & (0x0404 << ((1 - fChannelNumber) * 16)))
        f80PinCablePresent = true;

    /*f80PinCable[0] = ((readTimingRegister(kTimingRegUltra, 0) & 0x10) != 0);
    f80PinCable[1] = ((readTimingRegister(kTimingRegUltra, 1) & 0x10) != 0);*/


	// Get the base address for the bus master registers in I/O space.
    if ( getBMBaseAddress( fChannelNumber, &fBMBaseAddr ) != true )
    {
        DEBUG_LOG("%s: invalid bus-master base address\n", getName());
        goto fail;
    }

    // Must setup these variables inherited from IOPCIATA before it is started.

    _bmCommandReg   = IOATAIOReg8::withAddress( fBMBaseAddr + BM_COMMAND );
    _bmStatusReg    = IOATAIOReg8::withAddress( fBMBaseAddr + BM_STATUS );
    _bmPRDAddresReg = IOATAIOReg32::withAddress( fBMBaseAddr + BM_PRD_TABLE );
	
	// Make sure the compiler doesn't drop this line, or driver will be busy all the time :)
	
	fInterryptStatus = IOATAIOReg32::withAddress( fBMBaseAddr + 0x440 );

    // Reset bus timings for both drives.

    initializeHardware();
    resetBusTimings();

    // Override P-ATA reporting in IOATAController::start()
    // for SystemProfiler.

    if (fProvider->getHardwareType() == PCI_HW_SATA)
    {
        setProperty( kIOPropertyPhysicalInterconnectTypeKey,
                     kIOPropertyPhysicalInterconnectTypeSerialATA );
    }


    // Now we are ready to call super::start

    if ( super::start(_provider) == false )
    {
        goto fail;
    }
    superStarted = true;

    // This driver will handle interrupts using a work loop.
    // Create interrupt event source that will signal the
    // work loop (thread) when a device interrupt occurs.

	if ( fProvider->getInterruptVector() == 14 ||
         fProvider->getInterruptVector() == 15 )
    {
        // Legacy IRQ are never shared, no need for an interrupt filter.

        fInterruptSource = IOInterruptEventSource::interruptEventSource(
                           this, &interruptOccurred,
                           fProvider, 0 );
    }
    else
    {
		fInterruptSource = IOFilterInterruptEventSource::filterInterruptEventSource(
                           this, &interruptOccurred, &interruptFilter,
                           fProvider, 0 );
	}

    if ( !fInterruptSource ||
         (fWorkLoop->addEventSource(fInterruptSource) != kIOReturnSuccess) )
    {
        DEBUG_LOG("%s: interrupt registration error\n", getName());
        goto fail;
    }
	
    fInterruptSource->enable();

    // Attach to power management.

    initForPM( provider );

    // For each device discovered on the ATA bus (by super),
    // create a nub for that device and call registerService() to
    // trigger matching against that device.

    for ( UInt32 i = 0; i < kMaxDriveCount; i++ )
    {
        if ( _devInfo[i].type != kUnknownATADeviceType )
        {
            ATADeviceNub * nub;

            nub = ATADeviceNub::ataDeviceNub( (IOATAController*) this,
                                              (ataUnitID) i,
                                              _devInfo[i].type );

            if ( nub )
            {
                if ( _devInfo[i].type == kATAPIDeviceType )
                {
                    nub->setProperty( kIOMaximumSegmentCountReadKey,
                                      kATAMaxDMADesc / 2, 64 );

                    nub->setProperty( kIOMaximumSegmentCountWriteKey,
                                      kATAMaxDMADesc / 2, 64 );

                    nub->setProperty( kIOMaximumSegmentByteCountReadKey,
                                      0x10000, 64 );

                    nub->setProperty( kIOMaximumSegmentByteCountWriteKey,
                                      0x10000, 64 );
                }

                if ( nub->attach( this ) )
                {
                    _nub[i] = (IOATADevice *) nub;
                    _nub[i]->retain();
                    _nub[i]->registerService();
                }
                nub->release();
            }
        }
    }

    // Successful start, announce useful properties.

	IOLog("%s: NVIDIA nForce by medevil (CMD 0x%x, CTR 0x%x, IRQ %ld, BM 0x%x)\n", getName(),
          fProvider->getCommandBlockAddress(),
          fProvider->getControlBlockAddress(),
          fProvider->getInterruptVector(),
          fBMBaseAddr);

    return true;

fail:
    if ( fProvider )
        fProvider->close( this );

    if ( superStarted )
        super::stop( provider );

    return false;
}

/*---------------------------------------------------------------------------
 *
 * Stop the single-channel NVIDIA ATA controller driver.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::stop( IOService * provider )
{
    PMstop();
    super::stop( provider );
}

/*---------------------------------------------------------------------------
 *
 * Release resources before this driver is destroyed.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::free( void )
{
#define RELEASE(x) do { if(x) { (x)->release(); (x) = 0; } } while(0)

    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);

    // Release resources created by start().

    if (fInterruptSource && fWorkLoop)
    {
        fWorkLoop->removeEventSource(fInterruptSource);
    }

    RELEASE( fProvider        );
    RELEASE( fInterruptSource );
    RELEASE( fWorkLoop        );
    RELEASE( _nub[0]          );
    RELEASE( _nub[1]          );
    RELEASE( _bmCommandReg    );
    RELEASE( _bmStatusReg     );
    RELEASE( _bmPRDAddresReg  );
	RELEASE( fInterruptSource );

    // Release registers created by configureTFPointers().

    RELEASE( _tfDataReg       );
    RELEASE( _tfFeatureReg    );
    RELEASE( _tfSCountReg     );
    RELEASE( _tfSectorNReg    );
    RELEASE( _tfCylLoReg      );
    RELEASE( _tfCylHiReg      );
    RELEASE( _tfSDHReg        );
    RELEASE( _tfStatusCmdReg  );
    RELEASE( _tfAltSDevCReg   );

    // IOATAController should release this.

    if ( _doubleBuffer.logicalBuffer )
    {
        IOFree( (void *) _doubleBuffer.logicalBuffer,
                         _doubleBuffer.bufferSize );
        _doubleBuffer.bufferSize     = 0;
        _doubleBuffer.logicalBuffer  = 0;
        _doubleBuffer.physicalBuffer = 0;
    }

    // What about _cmdGate, and _timer in the superclass?

    super::free();
}

/*---------------------------------------------------------------------------
 *
 * Return the driver's work loop
 *
 ---------------------------------------------------------------------------*/

IOWorkLoop * AppleNForceATA::getWorkLoop( void ) const
{
    return fWorkLoop;
}

/*---------------------------------------------------------------------------
 *
 * Override IOATAController::synchronousIO()
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::synchronousIO( void )
{
    // IOATAController::synchronousIO() asserts nIEN bit in order to disable
    // drive interrupts during polled mode command execution. The problem is
    // that this will float the INTRQ line and put it in high impedance state,
    // which on certain systems has the undesirable effect of latching a false
    // interrupt on the interrupt controller. Perhaps those systems lack a
    // strong pull down resistor on the INTRQ line. Experiment shows that the
    // interrupt event source is signalled, and its producerCount incremented
    // after every synchronousIO() call. This false interrupt can become
    // catastrophic after reverting to async operations since software can
    // issue a command, handle the false interrupt, and issue another command
    // to the drive before the actual completion of the first command, leading
    // to a irrecoverable bus hang. This function is called after an ATA bus
    // reset. Waking from system sleep will exercise this path.
    // The workaround is to mask the interrupt line while the INTRQ line is
    // floating (or bouncing).
	
	IOReturn err = kATANoErr;

    if (fInterruptSource) fInterruptSource->disable();
	err = super::synchronousIO();
    if (fInterruptSource) fInterruptSource->enable();

    return err;
}

/*---------------------------------------------------------------------------
 *
 * Determine the start of the I/O mapped Bus-Master registers.
 *
 ---------------------------------------------------------------------------*/

bool AppleNForceATA::getBMBaseAddress( UInt32   channel,
                                          UInt16 * baseAddr )
{
    UInt32 bmiba;

    DEBUG_LOG("%s::%s( %p, %ld, %p )\n", getName(), __FUNCTION__,
              this, channel, baseAddr);

    bmiba = fProvider->pciConfigRead32( PCI_BMIBA );

    if ((bmiba & PCI_BMIBA_RTE) == 0)
    {
        DEBUG_LOG("%s: PCI BAR 0x%02x (0x%08lx) is not an I/O range\n",
                  getName(), PCI_BMIBA, bmiba);
        return false;
    }

    bmiba &= PCI_BMIBA_MASK;  // get the address portion
    if (bmiba == 0)
    {
        DEBUG_LOG("%s: BMIBA is zero\n", getName());
        return false;
    }

    if (channel == SEC_CHANNEL_ID)
        bmiba += BM_SEC_OFFSET;

    *baseAddr = (UInt16) bmiba;
    DEBUG_LOG("%s: BMBaseAddr = %04x\n", getName(), *baseAddr);

    return true;
}

/*---------------------------------------------------------------------------
 *
 * Reset all timing registers to the slowest (most compatible) timing.
 * DMA modes are disabled.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::resetBusTimings( void )
{
    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);

    memset(&fBusTimings[0], 0, sizeof(fBusTimings));

    fBusTimings[0].pioTiming = &PIOTimingTable[0];
    fBusTimings[1].pioTiming = &PIOTimingTable[0];

    programTimingRegisters();
}

/*---------------------------------------------------------------------------
 *
 * Setup the location of the task file registers.
 *
 ---------------------------------------------------------------------------*/

bool AppleNForceATA::configureTFPointers( void )
{
    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);

    UInt16 cmdBlockAddr = fProvider->getCommandBlockAddress();
    UInt16 ctrBlockAddr = fProvider->getControlBlockAddress();

    _tfDataReg      = IOATAIOReg16::withAddress( cmdBlockAddr + 0 );
    _tfFeatureReg   = IOATAIOReg8::withAddress(  cmdBlockAddr + 1 );
    _tfSCountReg    = IOATAIOReg8::withAddress(  cmdBlockAddr + 2 );
    _tfSectorNReg   = IOATAIOReg8::withAddress(  cmdBlockAddr + 3 );
    _tfCylLoReg     = IOATAIOReg8::withAddress(  cmdBlockAddr + 4 );
    _tfCylHiReg     = IOATAIOReg8::withAddress(  cmdBlockAddr + 5 );
    _tfSDHReg       = IOATAIOReg8::withAddress(  cmdBlockAddr + 6 );
    _tfStatusCmdReg = IOATAIOReg8::withAddress(  cmdBlockAddr + 7 );
    _tfAltSDevCReg  = IOATAIOReg8::withAddress(  ctrBlockAddr + 2 );
	
    if ( !_tfDataReg || !_tfFeatureReg || !_tfSCountReg ||
         !_tfSectorNReg || !_tfCylLoReg || !_tfCylHiReg ||
         !_tfSDHReg || !_tfStatusCmdReg || !_tfAltSDevCReg )
    {
        return false;
    }

    return true;
}

/*---------------------------------------------------------------------------
 *
 * The work loop based interrupt handler called by our interrupt event
 * source.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::handleDeviceInterrupt( void )
{
	if ( _currentCommand == 0 )
		return 0;
		
	IOReturn result = super::handleDeviceInterrupt();

	// clear the edge-trigger bit
	*_bmStatusReg = BM_STATUS_INT;
	OSSynchronizeIO();
	
	return result;
}

void AppleNForceATA::interruptOccurred( OSObject *               owner,
                                           IOInterruptEventSource * source,
                                           int                      count )
{
    AppleNForceATA * self = (AppleNForceATA *) owner;
	
    // Let our superclass handle the interrupt to advance to the next state
    // in the state machine.

    self->handleDeviceInterrupt();
}

/*---------------------------------------------------------------------------
 *
 * Filter interrupts that are not originated by our hardware. This will help
 * prevent waking up our work loop thread when sharing a interrupt line with
 * another driver.
 *
 ---------------------------------------------------------------------------*/

bool AppleNForceATA::interruptFilter( OSObject * owner,
                                         IOFilterInterruptEventSource * src )
{
    AppleNForceATA * self = (AppleNForceATA *) owner;
	
	return self->interruptIsValid( src );
}

bool AppleNForceATA::interruptIsValid( IOFilterInterruptEventSource * src )
{
	OSSynchronizeIO();
	
	if ( (*_bmStatusReg) & BM_STATUS_INT ) 
		return true;
		
	return false;
}

/*---------------------------------------------------------------------------
 *
 * Extend the implementation of scanForDrives() from IOATAController
 * to issue a soft reset before scanning for ATA/ATAPI drive signatures.
 *
 ---------------------------------------------------------------------------*/

UInt32 AppleNForceATA::scanForDrives( void )
{
    UInt32 unitsFound;

    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);

    *_tfAltSDevCReg = mATADCRReset;

    IODelay( 200 );

    *_tfAltSDevCReg = 0x0;

    IOSleep( 10 );

    unitsFound = super::scanForDrives();

    *_tfSDHReg = 0x00;  // Initialize device selection to device 0.

    return unitsFound;
}

/*---------------------------------------------------------------------------
 *
 * Provide information on the ATA bus capability.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::provideBusInfo( IOATABusInfo * infoOut )
{
    DEBUG_LOG("%s::%s( %p, %p )\n", getName(), __FUNCTION__, this, infoOut);

    if ( infoOut == 0 )
    {
        DEBUG_LOG("%s: %s bad argument\n", getName(), __FUNCTION__);
        return -1;
    }

    infoOut->zeroData();

    if (fProvider->getHardwareType() == PCI_HW_SATA)
        infoOut->setSocketType( kInternalSATA );
    else
        infoOut->setSocketType( kInternalATASocket );

    infoOut->setPIOModes( kPIOModeMask );
    infoOut->setDMAModes( kDMAModeMask );
    infoOut->setUltraModes( kUDMAModeMask );
    infoOut->setExtendedLBA( true );
    infoOut->setMaxBlocksExtended( 0x0800 );  // 2048 sectors for ext LBA

    UInt8 units = 0;
    if ( _devInfo[0].type != kUnknownATADeviceType ) units++;
    if ( _devInfo[1].type != kUnknownATADeviceType ) units++;
    infoOut->setUnits( units );

    return kATANoErr;
}

/*---------------------------------------------------------------------------
 *
 * Returns the currently configured timings for the drive unit.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::getConfig( IOATADevConfig * configOut,
                                       UInt32           unit )
{
    DEBUG_LOG("%s::%s( %p, %p, %ld )\n", getName(), __FUNCTION__,
              this, configOut, unit);

    if ((configOut == 0) || (unit > kATADevice1DeviceID))
    {
        DEBUG_LOG("%s: %s bad argument\n", getName(), __FUNCTION__);
        return -1;
    }

    configOut->setPIOMode( 0 );
    configOut->setDMAMode( 0 );
    configOut->setUltraMode( 0 );

    // Note that we need to report the bitmap of each mode,
    // not its mode number.

    if (TIMING_PARAM_IS_VALID(fBusTimings[unit].pioTiming))
    {
        configOut->setPIOMode( 1 << fBusTimings[unit].pioModeNumber );
        configOut->setPIOCycleTime( fBusTimings[unit].pioTiming->cycle );
    }

    if (TIMING_PARAM_IS_VALID(fBusTimings[unit].dmaTiming))
    {
        configOut->setDMAMode( 1 << fBusTimings[unit].dmaModeNumber );
        configOut->setDMACycleTime( fBusTimings[unit].dmaTiming->cycle );
    }

    if (fBusTimings[unit].ultraEnabled)
    {
        configOut->setUltraMode( 1 << fBusTimings[unit].ultraModeNumber );
    }

    configOut->setPacketConfig( _devInfo[unit].packetSend );

    return kATANoErr;
}

/*---------------------------------------------------------------------------
 *
 * Select the bus timings for a given drive unit.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::selectConfig( IOATADevConfig * configRequest,
                                          UInt32           unit )
{
    DEBUG_LOG("%s::%s( %p, %p, %ld )\n", getName(), __FUNCTION__,
              this, configRequest, unit);

    if ((configRequest == 0) || (unit > kATADevice1DeviceID))
    {
        DEBUG_LOG("%s: %s bad argument\n", getName(), __FUNCTION__);
        return -1;
    }

    // All config requests must include a supported PIO mode

    if ((configRequest->getPIOMode() & kPIOModeMask) == 0)
    {
        DEBUG_LOG("%s: PIO mode unsupported\n", getName());
        return kATAModeNotSupported;
    }

    if (configRequest->getDMAMode() & ~kDMAModeMask)
    {
        DEBUG_LOG("%s: DMA mode unsupported (0x%x)\n",
                  getName(), configRequest->getDMAMode());
        return kATAModeNotSupported;
    }

    if (configRequest->getUltraMode() & ~kUDMAModeMask)
    {
        DEBUG_LOG("%s: UDMA mode unsupported (0x%x)\n",
                  getName(), configRequest->getUltraMode());
        return kATAModeNotSupported;
    }

    if (configRequest->getDMAMode() && configRequest->getUltraMode())
    {
        DEBUG_LOG("%s: multiple DMA mode selection error\n", getName());
        return kATAModeNotSupported;
    }

    _devInfo[unit].packetSend = configRequest->getPacketConfig();

    selectTimingParameter( configRequest, unit );

    return getConfig( configRequest, unit );
}

/*---------------------------------------------------------------------------
 *
 * Select timing parameters based on config request.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::selectTimingParameter( IOATADevConfig * configRequest,
                                               UInt32           unit )
{
    DEBUG_LOG("%s::%s( %p, %d )\n", getName(), __FUNCTION__, this, (int)unit);

    // Reset existing parameters for this unit.

    fBusTimings[unit].pioTiming = &PIOTimingTable[0];
    fBusTimings[unit].dmaTiming = 0;
    fBusTimings[unit].ultraEnabled = false;

    if ( configRequest->getPIOMode() )
    {
        UInt32  pioModeNumber;
        UInt32  pioCycleTime;
        UInt32  pioTimingEntry = 0;

        pioModeNumber = bitSigToNumeric( configRequest->getPIOMode() );
        pioModeNumber = min(pioModeNumber, kPIOModeCount - 1);

        // Use a default cycle time if the device didn't report a time to use.
    
        pioCycleTime = configRequest->getPIOCycleTime();
        pioCycleTime = max(pioCycleTime, PIOMinCycleTime[pioModeNumber]);

        // Look for the fastest entry in the PIOTimingTable with a cycle time
        // which is larger than or equal to pioCycleTime.
    
        for (int i = kPIOTimingCount - 1; i > 0; i--)
        {
            if ( PIOTimingTable[i].cycle >= pioCycleTime )
            {
                pioTimingEntry = i;
                break;
            }
        }

        fBusTimings[unit].pioTiming = &PIOTimingTable[pioTimingEntry];
        fBusTimings[unit].pioModeNumber = pioModeNumber;
        DEBUG_LOG("%s: selected PIO mode %d\n", getName(), (int)pioModeNumber);
        setDriveProperty(unit, kSelectedPIOModeKey, pioModeNumber, 8);
    }

    if ( configRequest->getDMAMode() )
    {
        UInt32  dmaModeNumber;
        UInt32  dmaCycleTime;
        UInt32  dmaTimingEntry = 0;

        dmaModeNumber = bitSigToNumeric( configRequest->getDMAMode() );
        dmaModeNumber = min(dmaModeNumber, kDMAModeCount - 1);

        dmaCycleTime = configRequest->getDMACycleTime();
        dmaCycleTime = max(dmaCycleTime, DMAMinCycleTime[dmaModeNumber]);

        // Look for the fastest entry in the DMATimingTable with a cycle time
        // which is larger than or equal to dmaCycleTime.
    
        for (int i = kDMATimingCount - 1; i > 0; i--)
        {
            if ( DMATimingTable[i].cycle >= dmaCycleTime )
            {
                dmaTimingEntry = i;
                break;
            }
        }
        
        fBusTimings[unit].dmaTiming = &DMATimingTable[dmaTimingEntry];
        fBusTimings[unit].dmaModeNumber = dmaModeNumber;
        DEBUG_LOG("%s: selected DMA mode %d\n", getName(), (int)dmaModeNumber);
        setDriveProperty(unit, kSelectedDMAModeKey, dmaModeNumber, 8);
    }

    if ( configRequest->getUltraMode() )
    {
        UInt32  ultraModeNumber;

        ultraModeNumber = bitSigToNumeric( configRequest->getUltraMode() );
        ultraModeNumber = min(ultraModeNumber, kUDMAModeCount - 1);

        // For Ultra DMA mode 3 or higher, 80 pin cable must be present.
        // Otherwise, the drive will be limited to UDMA mode 2.

        if ( fProvider->getHardwareType() != PCI_HW_SATA && 
             ultraModeNumber > 2 )
        {
            if ( f80PinCablePresent == false )
            {
                DEBUG_LOG("%s: 80-conductor cable not detected\n", getName());
                ultraModeNumber = 2;
            }
        }

        fBusTimings[unit].ultraEnabled = true;
        fBusTimings[unit].ultraModeNumber = ultraModeNumber;
        DEBUG_LOG("%s: selected Ultra mode %d\n", getName(), (int)ultraModeNumber);
        setDriveProperty(unit, kSelectedUltraDMAModeKey, ultraModeNumber, 8);
    }

    programTimingRegisters();
}

/*---------------------------------------------------------------------------
 *
 * Program timing registers for both drives.
 *
 ---------------------------------------------------------------------------*/

static void mergeTimings( TimingParameter *       dst,
                          const TimingParameter * src )
{
    if (TIMING_PARAM_IS_VALID(dst) == false ||
        TIMING_PARAM_IS_VALID(src) == false)
        return;

    dst->cycle    = max(dst->cycle, src->cycle);
    dst->setup    = max(dst->setup, src->setup);
    dst->active   = max(dst->active, src->active);
    dst->recovery = max(dst->recovery, src->recovery);
}

void AppleNForceATA::programTimingRegisters( void )
{
    if (fProvider->getHardwareType() != PCI_HW_SATA)
    {
        TimingParameter  timingCommand;  // shared between both drives
        TimingParameter  timingData[2];

        memset(&timingCommand, 0, sizeof(timingCommand));
        memset(&timingData[0], 0, sizeof(timingData));

        for (int unit = 0; unit < 2; unit++)
        {
            if (DRIVE_IS_PRESENT(unit) == false)
                continue;

            mergeTimings( &timingCommand,    fBusTimings[unit].pioTiming );
            mergeTimings( &timingData[unit], fBusTimings[unit].pioTiming );
            mergeTimings( &timingData[unit], fBusTimings[unit].dmaTiming );
        }

        // We now have all the information need to program the registers.

        for (int unit = 0; unit < 2; unit++)
        {
            if (DRIVE_IS_PRESENT(unit) == false)
                continue;

            writeTimingIntervalNS( kTimingRegCommandActive,
                                unit, timingCommand.active );

            writeTimingIntervalNS( kTimingRegCommandRecovery,
                                unit, timingCommand.recovery );

            writeTimingIntervalNS( kTimingRegAddressSetup,
                                unit, timingData[unit].setup );

            writeTimingIntervalNS( kTimingRegDataActive,
                                unit, timingData[unit].active );

            writeTimingIntervalNS( kTimingRegDataRecovery,
                                unit, timingData[unit].recovery );

            if (fBusTimings[unit].ultraEnabled)
            {
                UInt8 mode = fBusTimings[unit].ultraModeNumber;
                writeTimingRegister( kTimingRegUltra, unit,
                                    UltraTimingTable[mode]); 
            }
            else
            {
                writeTimingRegister( kTimingRegUltra, unit, 0x8b ); 
            }        
        }
    }

    dumpHardwareRegisters();
}

/*---------------------------------------------------------------------------
 *
 * Read and write timing registers.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::writeTimingIntervalNS( TimingReg reg,
                                               UInt32       unit,
                                               UInt32       timeNS )
{
    const UInt32 clockPeriodPS = 30000; // 30ns @ 33MHz PCI
    UInt32 periods = ((timeNS * 1000) + clockPeriodPS - 1) / clockPeriodPS;
	UInt32 shifts = TimingRegInfo[reg].shift;
	
	if (reg == kTimingRegAddressSetup)
        shifts -= ((unit << 1) + (fChannelNumber << 2));

    periods =  min(periods, TimingRegInfo[reg].maxValue);
    periods =  max(periods, TimingRegInfo[reg].minValue);
    periods -= TimingRegInfo[reg].minValue;
    periods &= TimingRegInfo[reg].mask;
	periods <<= shifts;

    fProvider->pciConfigWrite8(TimingRegOffset[reg][fChannelNumber][unit],
                                 periods, TimingRegInfo[reg].mask << shifts);

    DEBUG_LOG("%s: CH%d DRV%d wrote 0x%02x to offset 0x%02x\n",
              getName(), (int)fChannelNumber, (int)unit, (unsigned int)periods,
              TimingRegOffset[reg][fChannelNumber][unit]);
}

void AppleNForceATA::writeTimingRegister( TimingReg reg,
                                             UInt32       unit,
                                             UInt8        periods )
{
    fProvider->pciConfigWrite8( TimingRegOffset[reg][fChannelNumber][unit],
                                periods, 0xFF );

    DEBUG_LOG("%s: CH%d DRV%d wrote 0x%02x to offset 0x%02x\n",
              getName(), (int)fChannelNumber, (int)unit, (unsigned int)periods,
              TimingRegOffset[reg][fChannelNumber][unit]);
}

UInt32 AppleNForceATA::readTimingIntervalNS( TimingReg reg, UInt32 unit )
{
    UInt32 time;
    UInt32 shifts = TimingRegInfo[reg].shift;

    if (reg == kTimingRegAddressSetup)
        shifts -= ((unit << 1) + (fChannelNumber << 2));

    time =   readTimingRegister( reg, unit );
    time >>= shifts;
	time &=  TimingRegInfo[reg].mask;
    time +=  TimingRegInfo[reg].minValue;
	time *=  30;

    return time;
}

UInt8 AppleNForceATA::readTimingRegister( TimingReg reg, UInt32 unit )
{
    return fProvider->pciConfigRead8(
                      TimingRegOffset[reg][fChannelNumber][unit]);
}

/*---------------------------------------------------------------------------
 *
 * Hardware initialization.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::initializeHardware( void )
{
    // Turn on prefetch and post write buffers for both primary and
    // secondary channels.

    fProvider->pciConfigWrite8( PCI_IDE_CONFIG, 0xF0, 0xF0 );
}

/*---------------------------------------------------------------------------
 *
 * Dynamically select the bus timings for a drive unit.
 *
 ---------------------------------------------------------------------------*/

void AppleNForceATA::selectIOTiming( ataUnitID unit )
{
    /* Timings was already applied by selectConfig() */
}

/*---------------------------------------------------------------------------
 *
 * Flush the outstanding commands in the command queue.
 * Implementation borrowed from MacIOATA in IOATAFamily.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::handleQueueFlush( void )
{
    UInt32 savedQstate = _queueState;

    DEBUG_LOG("%s::%s()\n", getName(), __FUNCTION__);

    _queueState = IOATAController::kQueueLocked;

    IOATABusCommand * cmdPtr = 0;

    while ( cmdPtr = dequeueFirstCommand() )
    {
        cmdPtr->setResult( kIOReturnError );
        cmdPtr->executeCallback();
    }

    _queueState = savedQstate;

    return kATANoErr;
}

/*---------------------------------------------------------------------------
 *
 * Handle termination notification from the provider.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleNForceATA::message( UInt32      type,
                                      IOService * provider,
                                      void *      argument )
{
    if ( ( provider == fProvider ) &&
         ( type == kIOMessageServiceIsTerminated ) )
    {
        fProvider->close( this );
        return kIOReturnSuccess;
    }

    return super::message( type, provider, argument );
}

/*---------------------------------------------------------------------------
 *
 * Publish a numeric property pertaining to a drive to the registry.
 *
 ---------------------------------------------------------------------------*/

bool AppleNForceATA::setDriveProperty( UInt32       driveUnit,
                                          const char * key,
                                          UInt32       value,
                                          UInt32       numberOfBits)
{
    char keyString[40];
    
    snprintf(keyString, 40, "Drive %ld %s", driveUnit, key);
    
    return super::setProperty( keyString, value, numberOfBits );
}

//---------------------------------------------------------------------------

IOReturn AppleNForceATA::createChannelCommands( void )
{
    IOMemoryDescriptor* descriptor = _currentCommand->getBuffer();
    IOMemoryCursor::PhysicalSegment physSegment;
    UInt32 index = 0;
    UInt8  *xferDataPtr, *ptr2EndData, *next64KBlock, *starting64KBlock;
    UInt32 xferCount, count2Next64KBlock;
    
    if ( !descriptor )
    {
        return -1;
    }

    // This form of DMA engine can only do 1 pass.
    // It cannot execute multiple chains.

    IOByteCount bytesRemaining = _currentCommand->getByteCount() ;
    IOByteCount xfrPosition    = _currentCommand->getPosition() ;
    IOByteCount  transferSize  = 0; 

    // There's a unique problem with pci-style controllers, in that each
    // dma transaction is not allowed to cross a 64K boundary. This leaves
    // us with the yucky task of picking apart any descriptor segments that
    // cross such a boundary ourselves.  

    while ( _DMACursor->getPhysicalSegments(
                           /* descriptor */ descriptor,
                           /* position   */ xfrPosition,
                           /* segments   */ &physSegment,
                           /* max segs   */ 1,
                           /* max xfer   */ bytesRemaining,
                           /* xfer size  */ &transferSize) )
    {
        xferDataPtr = (UInt8 *) physSegment.location;
        xferCount   = physSegment.length;

        if ( (UInt32) xferDataPtr & 0x01 )
        {
            IOLog("%s: DMA buffer %p not 2 byte aligned\n",
                  getName(), xferDataPtr);
            return kIOReturnNotAligned;        
        }

        if ( xferCount & 0x01 )
        {
            IOLog("%s: DMA buffer length %ld is odd\n",
                  getName(), xferCount);
        }

        // Update bytes remaining count after this pass.
        bytesRemaining -= xferCount;
        xfrPosition += xferCount;
            
        // Examine the segment to see whether it crosses (a) 64k boundary(s)
        starting64KBlock = (UInt8*) ( (UInt32) xferDataPtr & 0xffff0000);
        ptr2EndData  = xferDataPtr + xferCount;
        next64KBlock = starting64KBlock + 0x10000;

        // Loop until this physical segment is fully accounted for.
        // It is possible to have a memory descriptor which crosses more
        // than one 64K boundary in a single span.
        
        while ( xferCount > 0 )
        {
            if (ptr2EndData > next64KBlock)
            {
                count2Next64KBlock = next64KBlock - xferDataPtr;
                if ( index < kATAMaxDMADesc )
                {
                    setPRD( xferDataPtr, (UInt16)count2Next64KBlock,
                            &_prdTable[index], kContinue_PRD);
                    
                    xferDataPtr = next64KBlock;
                    next64KBlock += 0x10000;
                    xferCount -= count2Next64KBlock;
                    index++;
                }
                else
                {
                    IOLog("%s: PRD table exhausted error 1\n", getName());
                    _dmaState = kATADMAError;
                    return -1;
                }
            }
            else
            {
                if (index < kATAMaxDMADesc)
                {
                    setPRD( xferDataPtr, (UInt16) xferCount,
                            &_prdTable[index],
                            (bytesRemaining == 0) ? kLast_PRD : kContinue_PRD);
                    xferCount = 0;
                    index++;
                }
                else
                {
                    IOLog("%s: PRD table exhausted error 2\n", getName());
                    _dmaState = kATADMAError;
                    return -1;
                }
            }
        }
    } // end of segment counting loop.

    if (index == 0)
    {
        IOLog("%s: rejected command with zero PRD count (0x%lx bytes)\n",
              getName(), _currentCommand->getByteCount());
        return kATADeviceError;
    }

    // Transfer is satisfied and only need to check status on interrupt.
    _dmaState = kATADMAStatus;
    
    // Chain is now ready for execution.
    return kATANoErr;
}

//---------------------------------------------------------------------------

bool AppleNForceATA::allocDMAChannel( void )
{
    _prdTable = (PRD *) IOMallocContiguous(
                        /* size  */ sizeof(PRD) * kATAMaxDMADesc, 
                        /* align */ 0x10000, 
                        /* phys  */ &_prdTablePhysical );

    if ( !_prdTable )
    {
        IOLog("%s: PRD table allocation failed\n", getName());
        return false;
    }

    _DMACursor = IONaturalMemoryCursor::withSpecification(
                          /* max segment size  */ 0x10000,
                          /* max transfer size */ kMaxATAXfer );
    
    if ( !_DMACursor )
    {
        freeDMAChannel();
        IOLog("%s: Memory cursor allocation failed\n", getName());
        return false;
    }

    // fill the chain with stop commands to initialize it.    
    initATADMAChains( _prdTable );

    return true;
}

//---------------------------------------------------------------------------

bool AppleNForceATA::freeDMAChannel( void )
{
    if ( _prdTable )
    {
        // make sure the engine is stopped.
        stopDMA();

        // free the descriptor table.
        IOFreeContiguous(_prdTable, sizeof(PRD) * kATAMaxDMADesc);
    }

    return true;
}

//---------------------------------------------------------------------------

void AppleNForceATA::initATADMAChains( PRD * descPtr )
{
    UInt32 i;

    /* Initialize the data-transfer PRD channel command descriptors. */

    for (i = 0; i < kATAMaxDMADesc; i++)
    {
        descPtr->bufferPtr = 0;
        descPtr->byteCount = 1;
        descPtr->flags = OSSwapHostToLittleConstInt16( kLast_PRD );
        descPtr++;
    }
}

//---------------------------------------------------------------------------

enum {
    kPCIPowerStateOff = 0,
    kPCIPowerStateDoze,
    kPCIPowerStateOn,
    kPCIPowerStateCount
};

void AppleNForceATA::initForPM( IOService * provider )
{
    static const IOPMPowerState powerStates[ kPCIPowerStateCount ] =
    {
        { 1, 0, 0,             0,             0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0, IOPMSoftSleep, IOPMSoftSleep, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0, IOPMPowerOn,   IOPMPowerOn,   0, 0, 0, 0, 0, 0, 0, 0 }
    };

    PMinit();

    registerPowerDriver( this, (IOPMPowerState *) powerStates,
                         kPCIPowerStateCount );

    provider->joinPMtree( this );
}

//---------------------------------------------------------------------------

IOReturn AppleNForceATA::setPowerState( unsigned long stateIndex,
                                           IOService *   whatDevice )
{
    if ( stateIndex == kPCIPowerStateOff )
    {
        fHardwareLostContext = true;
    }
    else if ( fHardwareLostContext )
    {
        initializeHardware();
        programTimingRegisters();
        fHardwareLostContext = false;
    }

    return IOPMAckImplied;
}

//---------------------------------------------------------------------------

void AppleNForceATA::dumpHardwareRegisters( void )
{
    DEBUG_LOG("PCI_IDE_ENABLE   0x%02x\n", fProvider->pciConfigRead8(PCI_IDE_ENABLE));
    DEBUG_LOG("PCI_IDE_CONFIG   0x%02x\n", fProvider->pciConfigRead8(PCI_IDE_CONFIG));
    //DEBUG_LOG("PCI_CABLE_DETECT 0x%02x\n", fProvider->pciConfigRead8(PCI_CABLE_DETECT));
    DEBUG_LOG("PCI_FIFO_CONFIG  0x%02x\n", fProvider->pciConfigRead8(PCI_FIFO_CONFIG));
    DEBUG_LOG("PCI_ULTRA_TIMING 0x%08x\n", (unsigned int)fProvider->pciConfigRead32(PCI_ULTRA_TIMING));

    for (int unit = 0; unit < kMaxDriveCount; unit++)
    {
        if (DRIVE_IS_PRESENT(unit) == false) continue;

        DEBUG_LOG("[ Ch%ld Drive%ld ]\n", fChannelNumber, (long int)unit);
        DEBUG_LOG("Command Active   %ld ns\n", readTimingIntervalNS(kTimingRegCommandActive, unit));
        DEBUG_LOG("Command Recovery %ld ns\n", readTimingIntervalNS(kTimingRegCommandRecovery, unit));
        DEBUG_LOG("Address Setup    %ld ns\n", readTimingIntervalNS(kTimingRegAddressSetup, unit));
        DEBUG_LOG("Data Active      %ld ns\n", readTimingIntervalNS(kTimingRegDataActive, unit));
        DEBUG_LOG("Data Recovery    %ld ns\n", readTimingIntervalNS(kTimingRegDataRecovery, unit));

        if (fBusTimings[unit].ultraEnabled)
        {
            DEBUG_LOG("UDMA Timing      0x%02x\n", readTimingRegister(kTimingRegUltra, unit));
        }
    }
}

//---------------------------------------------------------------------------

IOReturn AppleNForceATA::writePacket( void )
{

	UInt32 packetSize = _currentCommand->getPacketSize();
	UInt16* packetData = _currentCommand->getPacketData();

	// First check if this ATAPI command requires a command packet…
	if ( packetSize == 0)						
	{
		return kATANoErr;
	}

	UInt8 status = 0x00;
		
	// While the drive is busy, wait for it to set DRQ.
	// limit the amount of time we will wait for a drive to set DRQ
	// ATA specs imply that all devices should set DRQ within 3ms. 
	// we will allow up to 30ms. (50ms)
	
	UInt32  breakDRQ = 5;

		
	while ( !waitForU8Status( (mATABusy | mATADataRequest), mATADataRequest)
			&& !checkTimeout()
			&& (breakDRQ != 0)  ) 
	{
		// check for a device abort - not legal under ATA standards,
		// but it could happen		
		status = *_tfAltSDevCReg;
		 //mask the BSY and ERR bits
		status &= (mATABusy | mATAError);

		// look for BSY=0 and ERR = 1
		if( mATAError == status )
		{
			DEBUG_LOG( "%s::%s: status without mATABusy | mATAError!", getName(), __FUNCTION__ );
			return kATADeviceError;
		}
		
		breakDRQ--;
		IOSleep( 10 );  // allow other threads to run
	 }

	// let the timeout through
	if ( checkTimeout() 
			|| breakDRQ == 0)
	{
		DEBUG_LOG( "%s::%s: timeout on waiting for DRQ.\n", getName(), __FUNCTION__ );
		return kATATimeoutErr;
	}
	// write the packet
	UInt32 packetLength = 6;
	
	if( packetSize > 12 )
	{
		packetLength = 8;
	
	}
	
	for( UInt32 i = 0; i < packetLength; i++)
	{
		OSSynchronizeIO();
		* _tfDataReg = *packetData;
		packetData++;	
	}

	UInt8 curStat = *_tfAltSDevCReg;

	if( _currentCommand->getFlags() & (mATAFlagUseDMA) )
	{
		
		while (!waitForU8Status( (mATADataRequest), 0) )
		{
			curStat = *_tfAltSDevCReg;
		}
			
		curStat = *_tfAltSDevCReg;
		activateDMAEngine();
	}
	
	return  kATANoErr ;

}

#pragma mark - IOATAController overrides (debug) -

/*---------------------------------------------------------------------------
 * true - bus is ready to dispatch commands
 * false - bus is busy with a current command.
 *
 ---------------------------------------------------------------------------*/
bool AppleNForceATA::busCanDispatch( void )
{

	// normal case
	if( _busState == IOATAController::kBusFree
		&& _queueState == IOATAController::kQueueOpen )
	{
		return true;
	}
	

	// special case if we are dispatching immediate commands only
	if( _busState == IOATAController::kBusFree
		&& _queueState == IOATAController::kQueueLocked
		&& _immediateGate == kImmediateOK 
		&& !queue_empty( &_commandQueue ) )
	{
	
		//DEBUG_LOG("IOATA Qfrozen check for immediate\n");
	
		// make sure the head of the queue is immediate
		IOATABusCommand* cmdPtr = (IOATABusCommand*) queue_first( & _commandQueue ) ;
		
		if( cmdPtr != 0 
			&& (cmdPtr->getFlags() & mATAFlagImmediate) )
		{
			//DEBUG_LOG("IOATA q-head is immediate\n");
			return true;
		}
	}
	
	
	// otherwise no dispatch.
	return false;

}

IOReturn AppleNForceATA::dispatchNext( void )
{
	
	IOReturn result = kATANoErr;

	//DEBUG_LOG("IOATAController::dispatchNext start\n");
	
	// check that the hardware is free and ready to accept commands
	if( !busCanDispatch() ) {
		//DEBUG_LOG( "IOATAController::dispatchNext: bus cannot dispatch!\n" );
		return result;
	}
	
	// set the bus state flag
	_busState = IOATAController::kBusBusy;
	
	// take the command at the head of the queue and make current.
	_currentCommand = dequeueFirstCommand();
	
	if( _currentCommand == 0L )
	{
		// if there's nothing in the queue, free the bus and 
		// return.
	
		//DEBUG_LOG("IOATAController::dispatchNext queue empty\n");
		_busState = IOATAController::kBusFree;
		return kATAQueueEmpty;
	
	}

	// set the state flag on the transaction.
	_currentCommand->state = IOATAController::kATAStarted;

	// IF we are in the special case circumstance of running commands 
	// that re-enter the command gate because they were dispatched during 
	// an executeEventCallout(), we MUST poll the device for command completion
	// This is an ugly artifact of the workloop design.
	
	if( _queueState == IOATAController::kQueueLocked
		&& _immediateGate == kImmediateOK
		&& _currentCommand->getFlags() & mATAFlagImmediate)
	{
	
		_currentCommand->setFlags(_currentCommand->getFlags() | mATAFlagUseNoIRQ );
	
	}

	// in case someone tries to slip a reset command through as an exec IO.

	if(	_currentCommand->getTaskFilePtr()->ataTFCommand == 0x08 )
	{
		DEBUG_LOG( "IOATAController::dispatchNext(): slip reset through exec IO!\n" );
		_currentCommand->setOpcode(kATAFnBusReset);
	}


	switch(	_currentCommand->getOpcode() )
	{
	
	
		case kATAFnExecIO:			/* Execute ATA I/O */
		case kATAPIFnExecIO:		/* ATAPI I/O */	
			result = handleExecIO();			
		break;

		case kATAFnRegAccess:		/* Register Access */	
			result = handleRegAccess();
		break;
		
		case kATAFnBusReset:		/* Reset ATA bus */
			result = handleBusReset();		
		break;

		case kATAFnQFlush:			/* I/O Queue Release */
			result = handleQueueFlush();
		break;
		
		
		default:
			DEBUG_LOG( "IOATAController::dispatchNext() unknown opcode: 0x%x.\n", _currentCommand->getOpcode() );
			_currentCommand->setResult( kATAUnknownOpcode );
			result = kATAUnknownOpcode;
			_currentCommand->state = IOATAController::kATAComplete;
			completeIO(kATAUnknownOpcode);
		break;	
	
	}

	//DEBUG_LOG("IOATAController::dispatchNext done return = %ld\n", (long int)result);

	return result;

}

IOReturn AppleNForceATA::handleExecIO( void )
{
	IOReturn err = kATANoErr;
	
	// select the desired device
	// don't start the IOTimer until after selection as there are no
	// generation counts in the IOTimerEventSource. Device Selection will honor 
	// the timeout value in ms on its own.
	err = selectDevice( _currentCommand->getUnit() );
	if( err )
	{	
		ERROR_LOG( "IOATAController device blocking bus.\n" );
		_currentCommand->state = IOATAController::kATAComplete;

		if( _currentCommand->getFlags() & mATAFlagUseNoIRQ )
		{
			completeIO( kIOReturnOffline );	
			return kIOReturnOffline;	
		}
		
		startTimer( 1000 );  // start a 1 second timeout so that we can unwind the stack if the bus is stuck.
		return kATANoErr;  // defer error handling to the timer thread. 
	}

	// start the IO Timer
	startTimer( _currentCommand->getTimeoutMS() );


	// go to asyncIO and start the state machine.
	// indicate the command has been issued
	_currentCommand->state = IOATAController::kATAStarted;		
	if( _currentCommand->getFlags() & mATAFlagUseNoIRQ )
	{
		err = synchronousIO();
	} else {
		err = asyncIO();
	}
		
	// return success and pend IRQ for further operation or completion.
	return err;
}

IOReturn AppleNForceATA::handleBusReset(void)
{

	bool		isATAPIReset = ((_currentCommand->getFlags() & mATAFlagProtocolATAPI) != 0);
	bool		doATAPI[2];
	IOReturn	err = kATANoErr;
	UInt8		index;
	UInt8 		statCheck;
	
	//DEBUG_LOG("IOATA bus reset start. (ATAPI reset? %s)\n", (isATAPIReset ? "yes" : "no") );

	doATAPI[0] = doATAPI[1] = false;		

	// If this is an ATAPI reset select just the corresponding atapi device (instead of both) 
	if (isATAPIReset)
	{
		doATAPI[_currentCommand->getUnit()] = true;  // Mark only selected ATAPI as reset victim.
	}
	else
	{
		doATAPI[0] = doATAPI[1] = true; // In ATA case, mark both as candidates for reset commands prior to a bus reset.
	}
	
	// Issue the needed ATAPI reset commands	
	for(index=0;index<2;index++)
	{
		if( doATAPI[index] && _devInfo[index].type == kATAPIDeviceType)
		{			
			OSSynchronizeIO();		
			*_tfSDHReg = mATASectorSize + (index << 4);

			// read the alt status and disreguard to provide 400ns delay
			OSSynchronizeIO();		
			statCheck = *_tfAltSDevCReg;  

			err = softResetBus(true);
		}
		
	}
	
	
	// once the ATAPI device has been reset, contact the device driver
	if (isATAPIReset)
	{
		DEBUG_LOG( "IOATAController::handleBusReset(): doing ATAPI reset for unit: %d\n", _currentCommand->getUnit() );
		executeEventCallouts( kATAPIResetEvent, _currentCommand->getUnit() );		
	}		
	

	// Handle the ATA reset case
	if(!isATAPIReset)
	{	
		err = softResetBus(); 	
		executeEventCallouts( kATAResetEvent, kATAInvalidDeviceID );	
	}
	
	_currentCommand->state = IOATAController::kATAComplete;

	//DEBUG_LOG("IOATAController::handleBusReset(): bus reset done.\n");

	completeIO( err );
	
	return err;
}

IOReturn AppleNForceATA::softResetBus( bool doATAPI )
{
	IOReturn result = kATANoErr;

	if (doATAPI)
	{	
		// ATAPI resets are directed to a device (0/1) which must be preselected
		// before entering this function.
	
		DEBUG_LOG("IOATAController::softResetBus() ATAPI reset command.\n");
		*_tfStatusCmdReg =  kSOFTRESET;				
		OSSynchronizeIO();
	
	} else {
		
		// begin the ATA soft reset sequence, which affects both devices on the 
		// bus
		
			
		// We will set nIEN bit to 0 to force the IRQ line to be driven by the selected
		// device.  We were seeing a small number of cases where the tristated line
		// caused false interrupts to the host.	

		*_tfAltSDevCReg = mATADCRReset;
		OSSynchronizeIO();						

		// ATA standards only dictate >5us of hold time for the soft reset operation
		// 100 us should be sufficient for any device to notice the soft reset.
		
		IODelay( 1000 );
		

		*_tfAltSDevCReg = 0x00;
		OSSynchronizeIO();
		
		DEBUG_LOG("IOATAController::softResetBus() soft reset sequenced (ATA)\n");
	
		// a reset operation has the effect of selecting device 0 as a result.
		// in this case, we will force our host controller to actually execute the 
		// device selection protocol before the next command.
		
		_selectedUnit = kATAInvalidDeviceID;
	
	}

	// ATA-4 and ATA-5 require the host to wait for >2ms 
	// after a sRST before sampling the drive status register.
	IOSleep(50);

	// ATA and ATAPI devices indicate reset completion somewhat differently
	// for ATA, wait for BSY=0 and RDY=1. For ATAPI, wait for BSY=0 only.
	UInt8 readyMask = mATABusy;
	UInt8 readyOn	= 0x00;
	
	if( (_devInfo[0].type == kATADeviceType)
		&& (!doATAPI) )
	{
		readyMask |= mATADriveReady;  //mask-in the busy + ready bit
		readyOn	= mATADriveReady;		// look for a RDY=1 as well as BSY=0
	}

	
	bool resetFailed = true;
	
	// loop for up to 31 seconds following a reset to allow 
	// drives to come on line. Most devices take 50-100ms, a sleeping drive 
	// may need to spin up and touch media to respond. This may take several seconds.
	for( int i = 0; i < 3100; i++)
	{
		// read the status register - helps deal with devices which errantly 
		// set interrupt pending states during resets. Reset operations are not 
		// supposed to generate interrupts, but some devices do anyway.
		// interrupt handlers should be prepared to deal with errant interrupts on ATA busses.
		OSSynchronizeIO();
		UInt8 status = *_tfStatusCmdReg;  	
		
		// when drive is ready, break the loop
		if( ( status & readyMask )== readyOn)
		{
			// device reset completed in time
			resetFailed = false;
			break;
		}
		
		IOSleep( 10 );  // sleep thread for another 10 ms
	
	}


	if ( resetFailed )
	{
		// it is likely that this hardware is broken. 
		// There's no recovery action if the drive fails 
		// to reset.	
		DEBUG_LOG("IOATAController::softResetBus() device failed to reset.\n");	
		result = kATATimeoutErr;
	}
	
	DEBUG_LOG("IOATAController::softResetBus() reset complete.\n");

	return result;

}

/*---------------------------------------------------------------------------
 *
 *  Perform device selection according to ATA standards document
 *	
 *
 ---------------------------------------------------------------------------*/
IOReturn AppleNForceATA::selectDevice( ataUnitID unit )
{
	UInt32 msLoops = _currentCommand->getTimeoutMS() / 10;
	
	/*if ( msLoops > 3000 )
		msLoops = 3000;*/

	// do a reality check
	if( ! (  (kATADevice0DeviceID == unit) 
			|| (kATADevice1DeviceID == unit ) ) )
	{
	
		DEBUG_LOG( "IOATA: invalid device ID selected\n");
		return kATAInvalidDevID;
	
	}		
	
	// give a chance for software to select the correct IO Timing 
	// in case the hardware doesn't maintain seperate timing regs 
	// for each device on the bus.
	selectIOTiming( unit );
	
	UInt8 preReqMask = ( mATABusy | mATADataRequest );
	UInt8 preReqCondition = 0x00;
	
	// if the device is already selected, no need to reselect it.
	// However, we do need to test for the correct status 
	// before allowing a command to continue. So we check for BSY=0 and DRQ=0
	// before selecting a device that isn't already selected first.
	
	// if the unit needs to be selected, test for a good bus and write the bit.
	if ( unit != _selectedUnit)
	{

		// check that BSY and DRQ are clear before selection
		while( !waitForU8Status( preReqMask, preReqCondition ) )
		{
			
			OSSynchronizeIO();
			if( msLoops == 0
				|| (*_tfStatusCmdReg & mATADataRequest ) == mATADataRequest // mATADataRequest) == mATADataRequest
				|| checkTimeout() )
			{
				DEBUG_LOG( "IOATA: BUSY or DRQ can't select device. \n");
				return kATAErrDevBusy;
			
			}
			msLoops--;
			IOSleep(10);  // allow other threads to run.
		}

		// invalide the currently selected device in case there's an error
		_selectedUnit = kATAInvalidDeviceID;
	
		// write the selection bit
		*_tfSDHReg = ( unit << 4 );
		OSSynchronizeIO();
	}	

	// unit was either selected above or was already the active device. Test for 
	// pre-requisite condition for commands.
	
	preReqMask = mATABusy;
	preReqCondition = 0x00;

	// for ATA devices, DRDY=1 is required with the exception of 
	// Init Drive Params and Execute Device diagnostics, 90h and 91h
	// as of ATA6, draft d1410r1a 
	// for Packet devices, DRDY is ignored for all commands.
	
	if( _devInfo[ unit ].type == kATADeviceType 
		&& _currentCommand->getOpcode() == kATAFnExecIO
		&& _currentCommand->getTaskFilePtr()->ataTFCommand != 0x90
		&& _currentCommand->getTaskFilePtr()->ataTFCommand != 0x91  )
	{
	
		preReqMask |= mATADriveReady;
		preReqCondition |= mATADriveReady;
	
	}
	
	// wait for BSY to clear
	msLoops = 10;
	 
	while( !waitForU8Status( preReqMask, preReqCondition ) ) // mATABusy, 0x00 ) ) 
	{
		
		OSSynchronizeIO();
		if( msLoops == 0
			|| (*_tfStatusCmdReg & mATADataRequest ) == mATADataRequest  //& (mATABusy | mATADataRequest) ) == mATADataRequest
			|| checkTimeout() )
		{
			DEBUG_LOG( "IOATA: BUSY can't select device. \n");
			return kATAErrDevBusy;
		
		}
		msLoops--;
		IOSleep(10);  // allow other threads to run.
	}

	// enable device interrupt
	OSSynchronizeIO();
	*_tfAltSDevCReg = 0x00;
	
	// successful device selection.
	_selectedUnit = unit;
	return kATANoErr;
}

IOReturn AppleNForceATA::asyncIO(void)
{
	IOReturn err = kATANoErr;

	if( (_currentCommand->getFlags() & mATAFlagProtocolATAPI) == mATAFlagProtocolATAPI
		&& _currentCommand->getPacketSize() > 0)
	{
		_currentCommand->state = determineATAPIState();
	}

	switch( _currentCommand->state )
	{
	
		
		case kATAStarted	:  // taskfile issue
			err = asyncCommand();
			//DEBUG_LOG("ATAController:: command sent: err = %ld state= %lx\n", (long int) err, (long unsigned int) _currentCommand->state);
			if( err )
			{
				_currentCommand->state = IOATAController::kATAComplete;
				asyncStatus();
				break;
			}

			// if next state isn't write packet, or if the packet device asserts IRQ,
			// return pending the next interrupt.
			if( _currentCommand->state != IOATAController::kATAPICmd
				|| _devInfo[ _currentCommand->getUnit() ].packetSend == kATAPIIRQPacket )
			{
				// pending IRQ
				//DEBUG_LOG("ATAController:: pending IRQ for packet\n");
				break;
			}
							
		// otherwise fall through and send the packet for DRQ devices.
		case kATAPICmd:	 // packet issue
			//DEBUG_LOG("ATAController:: issue packet\n");
			err = writePacket();
			if( err )
			{
				_currentCommand->state = IOATAController::kATAComplete;
				asyncStatus();
				break;
			}

			// if there's data IO, next phase is dataTx, otherwise check status.
			if( (_currentCommand->getFlags() & (mATAFlagIORead |  mATAFlagIOWrite ) )
				&&  ((_currentCommand->getFlags() & mATAFlagUseDMA ) != mATAFlagUseDMA ) )
			{
				_currentCommand->state = IOATAController::kATADataTx;
			}	else {  			
				// this is a non-data command, the next step is to check status.				
				_currentCommand->state = IOATAController::kATAStatus;			
			}

		break;
										
		case kATADataTx:  // PIO data transfer phase
			err = asyncData();
			if( err )
			{
				_currentCommand->state = IOATAController::kATAComplete;
				asyncStatus();
				break;
			}
		
			// if there's more data to transfer, then 
			// break. If ATA protocol PIO write, then break for IRQ
			if(_currentCommand->state == kATADataTx
				|| ( (_currentCommand->getFlags() & (mATAFlagProtocolATAPI | mATAFlagIOWrite | mATAFlagUseDMA) ) == mATAFlagIOWrite ) )
			{
				 break;
			}
			
			if( (_currentCommand->getFlags() & mATAFlagProtocolATAPI) == mATAFlagProtocolATAPI
				&& _currentCommand->getPacketSize() > 0)
			{			
				// atapi devices will go to status after an interrupt.
				break;			
			}	
				
			// else fall through to status state.
		case kATAStatus:  // data tx complete		
			err = asyncStatus();
			_currentCommand->state = IOATAController::kATAComplete;
		break;
	
		// state machine is somehow inconsistent	
		default:
			DEBUG_LOG("IOATA AsyncIO state broken\n");
			err = kATAErrUnknownType;
			_currentCommand->state = IOATAController::kATAComplete;
		break;
		
	}// end of switch ->state


	// call completeIO if the command is marked for completion.
	
	while (checkTimeout())
		IOSleep(10); //allow other threads to run.

	// BUG consider allowing the timeout to run rather than completing 
	// at this point. It might be safer to simply allow it to execute 
	// rather than move on the state machine at this point, since there's 
	// a possible race-condition if the timer expires while still inside the 
	// command gate.

	if( _currentCommand->state == IOATAController::kATAComplete )
	{
		completeIO(err);
	}
	
	return err;
}

IOReturn AppleNForceATA::startDMA( void )
{

	IOReturn err = kATANoErr;

	// first make sure the engine is stopped.
	stopDMA();
	
	
	// reality check the memory descriptor in the current command
	
	// state flag
	_dmaState = kATADMAStarting;
	
	// create the channel commands
	err = createChannelCommands();
	
	if(	err )
	{
	
		DEBUG_LOG("IOPCIATA error createChannelCmds err = %ld\n", (long int)err);
		stopDMA();
		return err;
	
	}
	
	// fire the engine
	activateDMAEngine();
	
	return err;

}

IOReturn AppleNForceATA::stopDMA( void )
{

	if(_dmaState != kATADMAInactive)
		shutDownATADMA();
	
	
	_dmaState = kATADMAInactive;
	return kATANoErr;

}

