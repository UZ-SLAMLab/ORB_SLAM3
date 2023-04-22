#include "ORBAccel.h"
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>

extern "C" {
	#include <libxlnk_cma.h>
}

using namespace std;

Buffer::Buffer() {}

Buffer::Buffer(int len) {
	max_length = len * sizeof(uint8_t);
	length = max_length;
	allocate();
}

void Buffer::allocate() {
	ptr = reinterpret_cast<uint8_t*>(cma_alloc(max_length, 0));
	phy_addr = cma_get_phy_addr(ptr);
}

void Buffer::copy(uchar* data, int len) {
	memcpy(ptr, data, len);
	length = len;
}

void Buffer::free() {
	cma_free(ptr);
}

MMIO::MMIO() {}

MMIO::MMIO(unsigned addr) {
	_base_addr = addr;
	int ret = init();
	if (ret < 0) {
		cout << "failed to initialize MMIO" << endl;
	}
}

void MMIO::close() {
	cma_munmap(_base_ptr, sizeof(unsigned)*24);
}

int MMIO::init() {
	_base_ptr = reinterpret_cast<unsigned*>(cma_mmap(_base_addr, sizeof(unsigned)*24)); 
	return 0;
}

unsigned MMIO::read(unsigned offset) {
	unsigned value = _base_ptr[offset >> 2];
	// cout << "MMIO::read, base: " << hex << _base_addr << ", offset: " << offset << ", value: " << value << endl;
	return value;
}

void MMIO::write(unsigned offset, unsigned value) {
	// cout << "MMIO::write, base: " << hex << _base_addr << ", offset: " << offset << ", value: 0x" << value << " | " << dec << value << endl;
	_base_ptr[offset >> 2] = value;
}

_SDMAChannel::_SDMAChannel() {}

_SDMAChannel::_SDMAChannel(MMIO mmio, int width, int tx_rx, int dre) {
	_mmio = mmio;
	_interrupt = 0;
	_max_size = (1 << MAX_LENGTH_WIDTH) - 1;
	_first_transfer = true;
	_align = width;
	_tx_rx = tx_rx;
	_dre = dre;
	if (_tx_rx == DMA_TYPE_RX) {
		_offset = 0x30;
		_flush_before = false;
	} else {
		_offset = 0x0;
		_flush_before = true;
	}
	transferred = 0;
	start();
}

bool _SDMAChannel::running() {
	// true if DMA engine is running
	return ((_mmio.read(_offset + 4) & 0x01) == 0x00);
}

bool _SDMAChannel::idle() {
	// true if DMA engine is idle
	return ((_mmio.read(_offset + 4) & 0x02) == 0x02);
}

bool _SDMAChannel::error() {
	// true if DMA engine is an error state
	return ((_mmio.read(_offset + 4) & 0x70) != 0x0);
}

void _SDMAChannel::stop() {
	// Stops the DMA channel and aborts the current transfer
	_mmio.write(_offset, 0x0000);
}

void _SDMAChannel::_clear_interrupt() {
	_mmio.write(_offset + 4, 0x1000);
}

void _SDMAChannel::start() {
	// Start the DMA engine if stopped
	if (_interrupt) {
		_mmio.write(_offset, 0x1001);
	} else {
		_mmio.write(_offset, 0x0001);
	}
	while (!running()) {
		continue;
	}
	_first_transfer = true;
}

void _SDMAChannel::transfer(Buffer* buffer) {
	transferred = 0;
	_mmio.write(_offset + 0x18, (buffer->phy_addr) & 0xFFFFFFFF);
	_mmio.write(_offset + 0x1C, ((buffer->phy_addr) >> 32) & 0xFFFFFFFF);
	_mmio.write(_offset + 0x28, buffer->length);
	_first_transfer = false;
}

void _SDMAChannel::wait() {
	if (!running()) {
		cout << "DMA channel not started" << endl;
		return;
	}
	while (true) {
		unsigned error = _mmio.read(_offset + 4);
		if (error) {
			if (error & 0x10) {
				cout << "DMA Internal Error (transfer length 0?)" << endl;
				return;
			}
			if (error & 0x20) {
				cout << "DMA Slave Error (cannot access memory map interface)" << endl;
				return;
			}
			if (error & 0x40) {
				cout << "DMA Decode Error (invalid address)" << endl;
				return;
			}
		}
		if (idle()) {
			break;
		}
	}
	transferred = _mmio.read(_offset + 0x28);
}

DMA::DMA() {}

DMA::DMA(unsigned addr) {
	_mmio = MMIO(addr);
	sendchannel = _SDMAChannel(_mmio, M_AXI_MM2S_DATA_WIDTH >> 3, DMA_TYPE_TX, MM2S_DRE);
	recvchannel = _SDMAChannel(_mmio, M_AXI_S2MM_DATA_WIDTH >> 3, DMA_TYPE_RX, S2MM_DRE);
}

unsigned DMA::read(unsigned offset) {
	return _mmio.read(offset);
}

void DMA::write(unsigned offset, unsigned value) {
	_mmio.write(offset, value);
}

void DMA::close() {
	_mmio.close();
}