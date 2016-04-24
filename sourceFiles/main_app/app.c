#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
#include <memory.h>

#include "PCIE.h"

//MAX BUFFER FOR DMA
#define MAXDMA 20

//BASE ADDRESS FOR CONTROL REGISTER
#define CRA 0x00000000

#define RWSIZE (32 / 8)

//	----------	Address parameters	------------

//BASE ADDRESS TO SDRAM
#define SDRAM 0x08000000

//	VGA base address
#define VGA_START_ADDR 0x08000000
//	VGA end address
#define VGA_END_ADDR 0x0812C000

//	Start byte address
#define START_BYTE_ADDR 0x0812D000
//	Start byte
#define START_BYTE 0xF00BF00B
//	Stop byte address
#define STOP_BYTE_ADDR 0x0812E000
//	Stop byte
#define STOP_BYTE 0xDEADF00B

//	Input image base address
#define INP_IMG_START_ADDR 0x09000000
//	Input image end address
#define INP_IMG_END_ADDR 0x0912C000

PCIE_BAR pcie_bars[] = { PCIE_BAR0, PCIE_BAR1 , PCIE_BAR2 , PCIE_BAR3 , PCIE_BAR4 , PCIE_BAR5 };

typedef struct tagBITMAPFILEHEADER
{
    WORD bfType;  //specifies the file type
    DWORD bfSize;  //specifies the size in bytes of the bitmap file
    WORD bfReserved1;  //reserved; must be 0
    WORD bfReserved2;  //reserved; must be 0
    DWORD bOffBits;  //species the offset in bytes from the bitmapfileheader to the bitmap bits
}BITMAPFILEHEADER;

typedef struct tagBITMAPINFOHEADER
{
    WORD dummy1;  //specifies the number of bytes required by the struct
    /*signed int*/ WORD  width;  //specifies width in pixels
    /*signed int*/ WORD dummy2;  //species height in pixels
    WORD height;
    DWORD dummy3;
    WORD biPlanes; //specifies the number of color planes, must be 1
    WORD biBitCount; //specifies the number of bit per pixel
    DWORD biCompression;//spcifies the type of compression
    DWORD biSizeImage;  //size of image in bytes
    signed int biXPelsPerMeter;  //number of pixels per meter in x axis
    signed int biYPelsPerMeter;  //number of pixels per meter in y axis
    DWORD biClrUsed;  //number of colors used by th ebitmap
    DWORD biClrImportant;  //number of colors that are important
}BITMAPINFOHEADER;

void test32( PCIE_HANDLE hPCIe, DWORD addr );
void testDMA( PCIE_HANDLE hPCIe, DWORD addr);

BOOL WriteStartByte(PCIE_HANDLE hPCIe);
BOOL ClearStartByte(PCIE_HANDLE hPCIe);

BOOL WriteImage(PCIE_HANDLE hPCIe, char *filename, BITMAPINFOHEADER *info);
BOOL WriteInfo2(PCIE_HANDLE hPCIe, BITMAPINFOHEADER *info);
BOOL checkImageDone(PCIE_HANDLE hPCIe);

BOOL clearMem(PCIE_HANDLE hPCIe, BITMAPINFOHEADER *info);
BOOL NewReadImage(PCIE_HANDLE hPCIe, BITMAPINFOHEADER *info);

void Demo(PCIE_HANDLE hPCIe, char *filename);

int main( int argc, char *argv[])
{
	//	Check number of arguments
	if(argc != 3 && argc != 2)
	{
	 	printf("Wrong command. Use **./app -h** for help.\n");
	 	return 0;
	}

	//	------------	PCIE stuff	------------
	
	//	Init library handle
	void *lib_handle;
	PCIE_HANDLE hPCIe;

	//	Load PCIE library handle
	lib_handle = PCIE_Load();

	//	Check PCIE library load
	if (!lib_handle)
	{
		printf("PCIE_Load failed\n");
		return 0;
	}

	//	Open PCIE handle
	hPCIe = PCIE_Open(0,0,0);

	//	Check PCIE handle
	if (!hPCIe)
	{
		printf("PCIE_Open failed\n");
		return 0;
	}

	//	------------	Arg Parsing	------------
	
	//	Check arguments
	char* input = argv[1];
	//	Demo
	if (strcmp("-d",input)==0)
	{
		//	Start main process
		Demo(hPCIe, argv[2]);
	}
	//	Help Message
	else if(strcmp("-h",input)==0)
	{
		printf("Use **./app -d <imagefilename>** to start demo.\n");	
	}
	//	Error Message
	else {
		printf("Wrong command. Use **./app -h** for help.\n");
	}

	return 0;
}

//	------------	MAIN PROCESS	------------

void Demo(PCIE_HANDLE hPCIe, char *filename)
{

	BITMAPINFOHEADER info;

	printf("\n\n");

	//	Clear start byte
	if(!ClearStartByte(hPCIe))
		return;

	//	Write Image to SDRAM
	if(!WriteImage(hPCIe, filename, &info))
		return; //dancing.bmp

	// //	Write Image header info to SDRAM
	// if(!WriteInfo2(hPCIe, &info))
	// 	return; //dancing.bmp

	//	Write start byte to start image processing
	if(!WriteStartByte(hPCIe))
		return;

	//	Check for stop byte done
	while(!checkImageDone(hPCIe));
	printf("\nProcessing finished.\n");

	// //	Create new image
	// if(!NewReadImage(hPCIe, &info))
	// 	return; //dancing.bmp

	// //	Clear memory
	// if(!clearMem(hPCIe, &info))
	// 	return;

	// printf("To copy the output image to your own account, use\n scp out.bmp mgXXX@ecegrid.ecn.purdue.edu:~/Desktop/. \n");
	// printf("\n\n");

	printf("Done Processing Image!");
	printf("\n\n");

	return;
}

// Clear STARTBYTE in SDRAM
BOOL ClearStartByte( PCIE_HANDLE hPCIe )
{
	DWORD addr 	= 	START_BYTE_ADDR;
	DWORD start = 	0x00000000;

	BOOL bPass 	= 	PCIE_Write32( hPCIe, pcie_bars[0], addr, start );

	//	Check success
	if( !bPass )
	{
		printf( "ERROR: unsuccessful clearing byte.\n" );
		return FALSE;
	}
	else
		printf( "Start byte cleared!\n" );

	return TRUE;
}

// Write STARTBYTE in SDRAM
BOOL WriteStartByte( PCIE_HANDLE hPCIe )
{
	DWORD addr 	= 	START_BYTE_ADDR;
	DWORD start = 	START_BYTE;

	BOOL bPass 	= 	PCIE_Write32( hPCIe, pcie_bars[0], addr, start );

	//	Check success
	if( !bPass )
	{
		printf( "ERROR: unsuccessful start byte writing.\n" );
		return FALSE;
	}
	else
		printf( "Start byte written.\n" );

	return TRUE;
}

// Write image info to slave register in user_module.sv
BOOL WriteInfo2( PCIE_HANDLE hPCIe, BITMAPINFOHEADER *info )
{

	printf("INFO: width = %d pixels; height = %d pixels\n", info -> width, info -> height);

	WORD tempw = info -> width;
	WORD temph = info -> height;

	DWORD addr = 0x04;
	//BYTE start = 0x04;
	PCIE_Write32( hPCIe, pcie_bars[0], addr, *( (unsigned char *) &tempw + 1 ) ); //01
	addr = addr + 4;
    PCIE_Write32( hPCIe, pcie_bars[0], addr, *( (unsigned char *) &tempw + 0 ) ); //f4
	addr = addr + 4;
	PCIE_Write32( hPCIe, pcie_bars[0], addr, *( (unsigned char *) &temph + 1 ) ); //01
	addr = addr + 4;
	//	Ask Lucas
	BOOL bPass = PCIE_Write32( hPCIe, pcie_bars[0], addr, *( ( unsigned char *) &temph + 0 ) ); //4d

	if( !bPass )
	{
		printf("ERROR: unsuccessful image info writing.\n");
		return FALSE;
	}
	else
		printf("Image info written.\n");
	return TRUE;
}

// Write the image field to SDRAM
BOOL WriteImage( PCIE_HANDLE hPCIe, char *filename, BITMAPINFOHEADER *info )
{
	//	Image File
	FILE * pFile;
 	pFile = fopen(filename,"rb");
 	//	Header buffer
	BITMAPFILEHEADER bitmapFileHeader; 
	//	image buffer
	unsigned char *bitmapImage;	

	//	Read bitmap header
  	fread(
  			&bitmapFileHeader, 
  			sizeof(BITMAPFILEHEADER),
  			1,
  			pFile
  		);

	//	Read the info header
	fread(
			info, 
			sizeof(BITMAPINFOHEADER),
			1,
			pFile
		);

	//	Allocate buffer space
	bitmapImage = (unsigned char*) malloc( info -> width * info -> height * sizeof( unsigned char ) );

	//	Check for allocation success
	if ( !bitmapImage )
   	{
   			// Deallocate space
        	free(bitmapImage);
        	fclose(pFile);
        	//	Error
			printf("Image loading failed.\n");
        	return FALSE;
    }

	//	Read in the bitmap image data
  	fread(	
  			bitmapImage,	//	Pointer to memory block
  			info -> width * info -> height * sizeof(unsigned char),	//	Size of each element
  			1,				//	Number of elements
  			pFile			//	Input Steam
  		);
	
	//BYTE tempRGB;
	DWORD addr = 0x09000000;  //original image written starting from 0x08000000
	// Write only one pixel to the LSByte and zero pad the rest 24 bits

	unsigned char *imageDataBuffer;
	imageDataBuffer = ( unsigned char* ) malloc( info -> width * info -> height * 4 * sizeof( unsigned char ) );
	int index = 0;
	int i;

	for( i = 0; i < info -> width * info -> height; ++i )
	{
		imageDataBuffer[index++] = 0;	//bitmapImage[i];
		imageDataBuffer[index++] = 0;
		imageDataBuffer[index++] = 0;
		imageDataBuffer[index++] = 255;
	}
	
	//	Write image buffer to the SDRAM
    BOOL bPass = PCIE_DmaWrite( hPCIe, addr, imageDataBuffer, info -> width * info -> height * 4 );

    //	Check success
	if(!bPass)
	{
		printf("ERROR: unsuccessful image writing.\n");
		return FALSE;
	}
	else
		printf("Image written.\n");

	//	Clear memory
	free(bitmapImage);
	free(imageDataBuffer);

	return TRUE;
}

// Check whether an image is finished by looking for STOPBYTE in slave register[0]
BOOL checkImageDone(PCIE_HANDLE hPCIe)
{
   BYTE b;
   DWORD addr = 0x00000000;
   BOOL bPass = PCIE_Read8( hPCIe, pcie_bars[0], addr, &b);
   BYTE check = 0x12;

   if(bPass)
   {
		if(b == check)
		{
			//printf("Image done\n");
			return TRUE;
		}
		else
		{
			//printf("Image not done yet\n");
			return FALSE;
		}
   }

   return FALSE;
}