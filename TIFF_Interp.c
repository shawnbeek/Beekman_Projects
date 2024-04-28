// Programmer: Shawn Beekman, 11/30/2023
// Computational Physics 5020 Final Project
/*	Use Bilinear Interpolation on a user input TIFF file to increase perceived image resolution
	First horizontally estimate, then vertically estimate, then handle the central pixels
	X's represent the sampled data (from initial TIF image), H's are the horizontal estimates, V's are the vertical estimates, C is the central estimate (average of the V's and H's)
	This unexpectedly happens to be simpler than handling the tiff library
	X H X ...
	V C V ...
	X H X ...
	. . .
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "tiffio.h"

// Function prototypes
uint32_t linearint(uint32_t color1, uint32_t color2);
uint32_t bilinearint(uint32_t color1, uint32_t color2, uint32_t color3, uint32_t color4);

int main() {
    uint32_t i,j,k;
    char inputfile[80], outfile[80];
    int ni;
    printf("Enter file to apply bilinear interpolation > ");
    ni = scanf("%s",inputfile);
    TIFF* tif_in = TIFFOpen(inputfile, "r");
    printf("Enter name of output file > ");
    ni = scanf("%s",outfile);
    printf("Processing...\n");
    /////// INPUT RASTER //////////
    uint32_t w, h;
    size_t npixels;
    uint32_t* raster;
    /////// OUTPUT RASTER /////////
    uint32_t newh, neww;
    size_t newnpixels;
    uint32_t* newraster;
	
    // Reads input .TIFF file and creates output raster
    if (tif_in) {
	TIFFGetField(tif_in, TIFFTAG_IMAGEWIDTH, &w);
	TIFFGetField(tif_in, TIFFTAG_IMAGELENGTH, &h);
	npixels = w * h;
	// New raster size is (*2-1) in each dimension
	neww = (2*w)-1;
	newh = (2*h)-1;
	newnpixels = neww * newh;
	raster = (uint32_t*) _TIFFmalloc(npixels * sizeof (uint32_t));
	newraster = (uint32_t*) _TIFFmalloc(newnpixels * sizeof (uint32_t));
	if (raster != NULL) {
	    if (TIFFReadRGBAImage(tif_in, w, h, raster, 0)) {
	    	// Pull input raster data into array for interpolation
	    	uint32_t array[newh][neww];
	    	k=0;
	    	for(i=newh-1;i>0;i-=2) {
	    		for(j=0;j<neww;j+=2) {
	    			//printf("array[%d][%d] = %X, k = %d\n",i,j,raster[k],k);
	    			array[i][j] = raster[k];
	    			k++; } 
	    	}
	    	for(j=0;j<neww;j+=2) {
	    			//printf("array[%d][%d] = %X, k = %d\n",0,j,raster[k],k);
	    			array[0][j] = raster[k];
	    			k++; 
	    	}
////////////////////////////////////////////////////////////////////////////////////////////////////
		// Interpolation section
	    	// Handle horizontal pixels
		for (i=0;i<=newh;i+=2) {
			for (j=1;j<neww;j+=2) {
				array[i][j] = linearint(array[i][j-1], array[i][j+1]); }
		}
		// Handle vertical pixels
		for (i=1;i<newh;i+=2) {
			for (j=0;j<=neww;j+=2) {
				array[i][j] = linearint(array[i-1][j], array[i+1][j]); }
		}
		
		// Handle center pixels
		for (i=1;i<newh;i+=2) {
			for (j=1;j<neww;j+=2) {
				array[i][j] = bilinearint(array[i][j-1], array[i][j+1], array[i-1][j], array[i+1][j]); }
		}
////////////////////////////////////////////////////////////////////////////////////////////////////
	    	// Deconstruct filled in array to newraster
	    	// Newraster is in the form of top right, left to right, top to bottom as elements ascend
	    	k = 0;
	    	for (i=0;i<=newh-1;i++) {
	    		for (j=0;j<=neww-1;j++) {
	    			newraster[k] = array[i][j];
	    			k++; }
	    	}
	    }
	    _TIFFfree(raster);
	    TIFFClose(tif_in);
	}
    }
   
    // Takes output raster and creates output.tiff
    TIFF* tif_out = TIFFOpen(outfile, "w");
    if (tif_out) {
        TIFFSetField(tif_out, TIFFTAG_IMAGEWIDTH, neww);
        TIFFSetField(tif_out, TIFFTAG_IMAGELENGTH, newh);
        TIFFSetField(tif_out, TIFFTAG_SAMPLESPERPIXEL, 4); // 4 channels (RGBA)
        TIFFSetField(tif_out, TIFFTAG_BITSPERSAMPLE, 8);   // 8 bits per channel
        TIFFSetField(tif_out, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
        TIFFSetField(tif_out, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
        TIFFSetField(tif_out, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);

        // Write the raster data
        if (TIFFWriteEncodedStrip(tif_out, 0, newraster, newnpixels * sizeof(uint32_t)) == -1) {
            fprintf(stderr, "Error writing image to %s\n", outfile);
        } else {
            printf("Done\nImage successfully written to %s\n", outfile);
        }
    } else {
        fprintf(stderr, "Could not open %s for writing\n", outfile);
    }
    
    _TIFFfree(newraster);
    TIFFClose(tif_out);
    return 0;
}

////////////// FUNCITONS ////////////////
// Linear Interpolation: Takes two array int values and averages them
uint32_t linearint(uint32_t color1, uint32_t color2) {
	// Interpolation of hex values is not as simple as it may seem
	// We must extract the channels of our unsigned int of 32 bits. The hex number 0xFF807060 for ex represents in this case Opacity (FF), Green (80), Blue (70), Red (60)
	uint8_t opacity1 = (color1 >> 24) & 0xFF; // shift MS2B to LSBs, AND away the crap so all that remains is opacity information
    	uint8_t green1 = (color1 >> 16) & 0xFF;
    	uint8_t blue1 = (color1 >> 8) & 0xFF;
    	uint8_t red1 = color1 & 0xFF;

    	uint8_t opacity2 = (color2 >> 24) & 0xFF;
    	uint8_t green2 = (color2 >> 16) & 0xFF;
    	uint8_t blue2 = (color2 >> 8) & 0xFF;
    	uint8_t red2 = color2 & 0xFF;

    	// Calculate averaged channels - simplified average
    	uint8_t avg_opacity = (opacity1 + opacity2) / 2;
    	uint8_t avg_green = (green1 + green2) / 2;
    	uint8_t avg_blue = (blue1 + blue2) / 2;
    	uint8_t avg_red = (red1 + red2) / 2;

    	// Construct the interpolated color
    	uint32_t result = (avg_opacity << 24) | (avg_green << 16) | (avg_blue << 8) | avg_red;

	return result;
}

// Bilinear Interpolation: Takes four array values surrounding the center pixel and applies linear interpolation to the avg horizontal and avg vertical values
uint32_t bilinearint(uint32_t color1, uint32_t color2, uint32_t color3, uint32_t color4) {
	uint32_t temp1 = linearint(color1, color2); // horizontal
	uint32_t temp2 = linearint(color3, color4); // vertical
	return (linearint(temp1, temp2));
}
