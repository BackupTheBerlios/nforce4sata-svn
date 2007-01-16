/*
 * Copyright (c) 2000-2004 medevil <medevil84@gmail.com> Apple Computer, Inc. All rights reserved.
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

/*
 *	nForceSATA.cpp
 *	
 *	Defines the concrete driver for a nForce controller. Supported
 *  ATA Controllers are defined in the nForceSATAHardware header.
 *	Descendent of IOPCIATA, which is derived from IOATAController.
 *
 */
 
#include "nForceSATAHardware.h"
#include "nForceSATA.h"

#include <IOKit/assert.h>
#include <IOKit/IOCommandGate.h>
#include <IOKit/IOTypes.h>
#include <IOKit/ata/ATADeviceNub.h>
#include <IOKit/ata/IOATATypes.h>
#include <IOKit/ata/IOATAController.h>
#include <IOKit/ata/IOATACommand.h>
#include <IOKit/ata/IOATADevice.h>
#include <IOKit/ata/IOATABusInfo.h>
#include <IOKit/ata/IOATADevConfig.h>

#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/storage/IOStorageProtocolCharacteristics.h>
#include <libkern/OSByteOrder.h>
#include <libkern/OSAtomic.h>

#define super IOPCIATA
OSDefineMetaClassAndStructors ( nForceSATA, IOPCIATA )

#pragma mark - IOService overrides -

bool nForceSATA::init( OSDictionary* properties )
{
	DEBUG_LOG( "nForceSATA: %s([%p] %p )\n", __FUNCTION__, this, properties );
	
	// reset base address map to null
	for (int i=0; i < 5; i++)
		fIOBaseAddrMap[i] = 0;
	
	fDeviceInterruptFilter = 0;
	fInterruptBitMask = 0;
	fBusChildNum = 0;
	
	if ( ! super::init( properties ) )
	{
		ERROR_LOG( "nForceSATA: %s() super::init( %p ) [this=%p] failed.\n", __FUNCTION__, properties, this );
		return false;
	}
	
	DEBUG_LOG( "nForceSATA: %s( %p ) [this=%p] completed.\n", __FUNCTION__, properties, this );
	
	return true;
}

IOService* nForceSATA::probe( IOService* provider,	SInt32*	score )
{
	DEBUG_LOG( "nForceSATA: %s( %p, %p ) [this=%p]\n", __FUNCTION__, provider, score, this );
	
	// DEBUG_LOG( "nForceSATA: %s( %p, %p ) [this=%p] completed.\n", __FUNCTION__, provider, score, this );
	
	return this;
}
	
bool nForceSATA::start( IOService* provider )
{
	ATADeviceNub* new_nub = 0L;
	
	DEBUG_LOG( "nForceSATA: %s( %p ) [this=%p]\n", __FUNCTION__, provider, this );
	
	if ( ! provider->open(this) )
	{
		ERROR_LOG( "nForceSATA: %s( %p ) [this=%p] provider did not open.\n", __FUNCTION__, provider, this );
		return false;
	}
	
	// set the connector key to SATA (this is mainly for SystemProfiler)
	setProperty( kIOPropertyPhysicalInterconnectTypeKey, 
		kIOPropertyPhysicalInterconnectTypeSerialATA );
		
	if ( ! super::start( provider ) )
	{
		// don't forget to close the provider!
		ERROR_LOG( "nForceSATA: %s( %p) [this=%p] super::start() failed.\n", __FUNCTION__, provider, this );
		provider->close(this);
		return false;
	}
	
	// create device interrupt
	fDeviceInterruptFilter = IOFilterInterruptEventSource::interruptEventSource(
					(OSObject*)this,
					(IOInterruptEventSource::Action) &nForceSATA::interruptOccurred,
					(IOFilterInterruptEventSource::Filter) &nForceSATA::interruptFilter,
					getProvider(), 
					0);
					
	if ( ! fDeviceInterruptFilter || getWorkLoop()->addEventSource(fDeviceInterruptFilter) )
	{
		ERROR_LOG( "nForceSATA: %s( %p) [this=%p] failed to create interrupt source.\n", __FUNCTION__, provider, this );
		super::stop();
		provider->close(this);
		return false;
	}
	
	fDeviceInterruptFilter->enable();
	
	// Attach the nub devices
	
	for (int i = 0; i < kMaxDriveCount; i++) 
	{
		if ( _devInfo[i].type != kUnknownATADeviceType )
		{
			DEBUG_LOG( "nForceSATA: %s( %p ) [this=%p] creating device nub %i.\n", __FUNCTION__, provider, this, i );
			
			new_nub = ATADeviceNub::ataDeviceNub(	(IOATAController*)this, 
													(ataUnitID) i, 
													(ataDeviceType) _devInfo[i].type );
			
			if ( new_nub )
			{
				new_nub->attach( this );
				
				_nub[i] = new_nub;
				
				new_nub->registerService();
				new_nub->release();
				
				DEBUG_LOG( "nForceSATA: %s( %p) [this=%p] nub device '%i' attached.\n", 
					__FUNCTION__, provider, this, i );
				
				new_nub = 0L;
			}
		}
	}
	
	DEBUG_LOG( "nForceSATA: %s( %p, %p ) [this=%p] bus '%i' completed.\n", 
		__FUNCTION__, provider, score, this, fBusChildNum );
	
	return true;
}

void nForceSATA::free( void )
{
	for (int i=0; i < 5; i++)
	{
		if ( fIOBaseAddrMap[i] )
		{
			fIOBaseAddrMap[i]->release();
			fIOBaseAddrMap[i] = 0;
		}
	}
	
	if ( fDeviceInterruptFilter )
	{
		fDeviceInterruptFilter->release();
		fDeviceInterruptFilter = 0;
	}
	
	super::free();
	
	return;
}

IOWorkLoop * nForceSATA::getWorkLoop( void ) const
{
	IOWorkLoop* wl = _workLoop;
	
	if ( !wl ) 
	{
		// Create a new workloop
		wl = IOWorkLoop::workLoop();
		
		if ( !wl ) 
			return 0;
	}
	
	return wl;
}

#pragma mark - IOATAController overrides -

bool nForceSATA::configureTFPointers( void )
{
	DEBUG_LOG( "nForceSATA: %s( void ) [this=%p]\n", __FUNCTION__, this );
	
	char locationStr[2] = { 0, 0 };
	sprintf(locationStr, "%1s", getProvider()->getLocation());
	
	//fBusChildNum = locationStr[0] - 0x30;
	
	//DEBUG_LOG( "nForceSATA: %s( void ) [this=%p] bus child num: %i [original: 0x%x]\n",
	//	__FUNCTION__, this, fBusChildNum, locationStr[0] );
	
	fIOBaseAddrMap[0] = getProvider()->mapDeviceMemoryWithIndex( 0 );
	
	if ( fIOBaseAddrMap[0] == 0 )
	{
		DEBUG_LOG( "nForceSATA: %s( void ) [this=%p] IOBaseAddrMap is zero.\n",
			__FUNCTION__, this );
		return false;
	}
	
	volatile UInt8* baseaddress = (volatile UInt8*)fIOBaseAddrMap[0]->getVirtualAddress();
	baseaddress += _selectedUnit * kATASecondaryCodecOffset;
	
	// in bsd code those variables are initialized with a 
	// for cycle that increments the offset by one
	_tfDataReg = (volatile UInt16*) (baseAddress + 0);
	_tfFeatureReg   = baseAddress + 0x4;
	_tfSCountReg    = baseAddress + 0x8;
	_tfSectorNReg   = baseAddress + 0xC;
	_tfCylLoReg     = baseAddress + 0x10;
	_tfCylHiReg     = baseAddress + 0x14;
	_tfSDHReg       = baseAddress + 0x18;
	_tfStatusCmdReg = baseAddress + 0x1C;
 	
	_tfAltSDevCReg  = baseAddress + 0x20;

	// control address
	_bmCommandReg   = (baseAddress + 0x58);
	_bmStatusReg    = (baseAddress + 0x5C);
	_bmPRDAddresReg = (volatile UInt32*) (baseAddress+ 0x60);
	
	// these should be correct!
	SATAStatus  = (volatile UInt32*) (baseAddress + 0x40); // these where 0x04	
	SATAError   = (volatile UInt32*) (baseAddress + 0x44);
	SATAControl = (volatile UInt32*) (baseAddress + 0x48);

//	SICR1 = (volatile UInt32*) (baseAddress + 0x80);
//	SICR2 = (volatile UInt32*) (baseAddress + 0x84);
	
}

IOReturn nForceSATA::provideBusInfo( IOATABusInfo* infoOut )
{
	if ( !infoOut )
	{
		ERROR_LOG( "nForceSATA: %s( %p ) [this=%p] nil provider for infoOut.\n", __FUNCTION__, infoOut, this );
		return (IOReturn)-1;
	}
	
	infoOut->zeroData();
	infoOut->setSocketType( kInternalSATA );
	infoOut->setPIOModes( kATASupportedPIOModes );
	infoOut->setDMAModes( 0 );
	infoOut->setUltraModes( kATASupportedUltraDMAModes );
	infoOut->setExtendedLBA( true );
	infoOut->setMaxBlocksExtended( 0x0800 );
	
	UInt8 units;
	
	for (int i=0; i < kMaxDriveCount; i++)
	{
		if (_devInfo[i].type != kUnknownATADeviceType )
			units++;
	}
	
	infoOut->setUnits( units );
	
	return kATANoErr;
}

IOReturn nForceSATA::getConfig( IOATADevConfig* configRequest, UInt32 unitNumber )
{
	if ( configRequest == NULL || unitNumber > kMaxDriveCount )
	{
		ERROR_LOG( "nForceSATA: %s( %p, %p ) [this=%p] nil config request or invalid unit number!\n", 
			__FUNCTION__, configRequest, unitNumber, this );
		return (IOReturn) -1;
	}
	
	configRequest->setPIOMode( fBusTimings[unitNumber].ataPIOSpeedMode );
	configRequest->setDMAMode( fBusTimings[unitNumber].ataMultiDMASpeed );
	configRequest->setUltraMode( fBusTimings[unitNumber].ataUltraDMASpeedMode );
	configRequest->setPIOCycleTime( fBusTimings[unitNumber].ataPIOCycleTime );
	configRequest->setDMACycleTime( fBusTimings[unitNumber].ataMultiCycleTime );
	
	configRequest->setPacketConfig( _devInfo[unitNumber].packetSend );
	
	return kATANoErr;
}

IOReturn nForceSATA::selectConfig( IOATADevConfig* configRequest, UInt32 unitNumber )
{

	if ( configRequest == NULL || unitNumber > kMaxDriveCount )
	{
		ERROR_LOG( "nForceSATA: %s( %p, %i ) [this=%p] nil config request or invalid unit number.\n", 
			__FUNCTION__, configRequest, unitNumber, this );
		return (IOReturn) -1;
	}
	
	if ( ( configRequest->getPIOMode() & kATASupportedPIOModes ) == 0x00 )
	{
		DEBUG_LOG( "nForceSATA: %s( %p, %i ) [this=%p] PIO mode not supported.\n",
			__FUNCTION__, configRequest, unitNumber, this );
		return kATAModeNotSupported;
	}
	
	if ( configRequest->getDMAMode() > 0 && configRequest->getUltraMode() > 0)
	{
		DEBUG_LOG( "nForceSATA: %s( %p, %i ) [this=%p] multiple DMA class selection not supported.\n",
			__FUNCTION__, configRequest, unitNumber, this );
		return kATAModeNotSupported;
	}
	
	// make sure a requested ultra ATA mode is within the range of this device configuration
	if ( configRequest->getUltraMode() & ~kATASupportedUltraDMAModes )
	{
		DEBUG_LOG( "nForceSATA: %s( %p, %i ) [this=%p] ultra DMA mode not supported.\n",
			__FUNCTION__, configRequest, unitNumber, this );
		return kATAModeNotSupported;	
	}
	
	// check also the multi DMA mode
	if ( configRequest->getDMAMode() & ~kATASupportedMultiDMAModes )
	{
		DEBUG_LOG( "nForceSATA: %s( %p, %i ) [this=%p] multi DMA mode not supported.\n",
			__FUNCTION__, configRequest, unitNumber, this );
		return kATAModeNotSupported;
	}
	
	_devInfo[unitNumber].packetSend = configRequest->getPacketConfig();
	DEBUG_LOG( "nForceSATA: %s( %p, %i ) [this=%p] packet send: %d\n",
		__FUNCTION__, configRequest, unitNumber, this );
		
	return kATANoErr;

}

void nForceSATA::handleTimeout( void )
{
	// stop the controller
	
	super::handleTimeout();
}

IOReturn nForceSATA::synchronousIO(void)
{
}

IOReturn nForceSATA::selectDevice( ataUnitID unit )
{
}

IOReturn nForceSATA::issueCommand( void )
{
}

IOReturn nForceSATA::writePacket( void )
{
}

IOReturn nForceSATA::softResetBus( bool doATAPI = false )
{
}

bool nForceSATA::ATAPISlaveExists( void )
{
}

UInt32 nForceSATA::scanForDrives( void )
{
}

IOReturn nForceSATA::registerAccess(bool isWrite)
{
}

void nForceSATA::selectIOTiming( ataUnitID unit )
{
}

#pragma mark - IOPCIATA optional overrides -
IOReturn nForceSATA::startDMA( void )
{
}

IOReturn nForceSATA::stopDMA( void )
{
}

bool nForceSATA::allocDMAChannel( void )
{
}

void nForceSATA::initATADMAChains( PRD* descPtr )
{
}

IOReturn nForceSATA::createChannelCommands( void )
{
}

bool nForceSATA::freeDMAChannel( void )
{
}

#pragma mark - Interrupt functions -
IOReturn nForceSATA::handleDeviceInterrupt(void)
{
}

void nForceSATA::interruptOccurred( OSObject* owner, IOInterruptEventSource* src, int count )
{
}

bool nForceSATA::interruptFilter( OSObject* owner, IOFilterInterruptEventSource* src )
{
}

#pragma mark - misc functions -

char* nForceSATA::getChipsetName(int chipsetIndex)
{
	int len = sizeof(nforceChipsets) / sizeof(nForceSATAChipset);
	
	if (chipsetIndex >= 0 && chipsetIndex < len)
		return nforceChipsets[chipsetIndex].text;
		
	return nforceChipsets[len-1].text;
}

int nForceSATA::getChipsetIndex(int vendor_id, int device_id)
{
	int index;
	int len = sizeof(nforceChipsets) / sizeof(nForceSATAChipset);
	
	UInt32 vendordevice_id = (device_id << 16) + vendor_id;
	
	for (index = 0; index < len; index++)
		if (nforceChipsets[index].chipid == vendordevice_id)
			return index;
	
	return -1;
}
