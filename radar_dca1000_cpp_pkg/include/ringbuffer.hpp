#ifndef RING_BUFFER_HPP
#define RING_BUFFER_HPP

class RingBuffer
{
public:
	RingBuffer( int sizeBytes );
    RingBuffer();
	~RingBuffer();
	int Read( unsigned char* dataPtr, int numBytes );
	int Write( unsigned char *dataPtr, int numBytes );
	bool Empty( void );
	int GetSize( );
	int GetWriteAvail( );
	int GetReadAvail( );
private:
	unsigned char * _data;
	int _size;
	int _readPtr;
	int _writePtr;
	int _writeBytesAvail;
};

#endif