#include <cstdint>

#ifndef _ORBACCEL_H_
#define _ORBACCEL_H_

#define ORB_BASE        0xA0010000
#define ORB_START       0x00
#define ORB_RETURN      0x10
#define ORB_HEIGHT      0x18
#define ORB_WIDTH       0x20
#define ORB_HEIGHT_NEW  0x28
#define ORB_WIDTH_NEW   0x30
#define ORB_THRESHOLD   0x38
#define OUTPUT_BYTES    36
#define MAX_OUTPUT_LENGTH 100000

#define DMA_BASE    0xA0000000
#define MM2S_DMACR  0x00
#define MM2S_DMASR  0x04
#define S2MM_DMACR  0x30
#define S2MM_DMASR  0x34

#define MAX_LENGTH_WIDTH 26
#define DMA_TYPE_TX 1
#define DMA_TYPE_RX 0
#define MM2S_DRE 0
#define S2MM_DRE 0
#define M_AXI_MM2S_DATA_WIDTH 128
#define M_AXI_S2MM_DATA_WIDTH 128

typedef unsigned char uchar;

class Buffer {
    public:
        Buffer();
        Buffer(int len);
        void allocate();
        void copy(uchar* data, int length);
        void free();
        int max_length;
        int length;
        unsigned long phy_addr;
        uint8_t *ptr;
};

class MMIO {
	public:
        MMIO();
		MMIO(unsigned addr);
		int init();
		unsigned read(unsigned offset);
		void write(unsigned offset, unsigned value);
        void close();
	private:
		unsigned _base_addr;
		int _fd;
		void *_ptr;
		unsigned _page_addr;
		unsigned _page_offset;
		unsigned _page_size;
        unsigned *_base_ptr;
};

class _SDMAChannel {
    public:
        _SDMAChannel();
        _SDMAChannel(MMIO mmio, int width, int tx_rx, int dre);
        void start();
		bool running();
		bool idle();
		bool error();
		void stop();
		void _clear_interrupt();
        void transfer(Buffer* buffer);
        void wait();
        int transferred;
    private:
        MMIO _mmio;
        int _interrupt;
        int _max_size;
        int _align;
        bool _first_transfer;
        int _tx_rx;
        int _dre;
		unsigned _offset;
		bool _flush_before;  
};

class DMA {
    public:
        DMA();
        DMA(unsigned addr);
        unsigned read(unsigned offset);
        void write(unsigned offset, unsigned value);
        void close();
		_SDMAChannel sendchannel;
		_SDMAChannel recvchannel;
	private:
		MMIO _mmio;
};

#endif