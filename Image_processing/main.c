

#include <stdio.h>
#include "I2C_core.h"
#include "terasic_includes.h"
#include "mipi_camera_config.h"
#include "mipi_bridge_config.h"

#include "auto_focus.h"

#include <fcntl.h>
#include <unistd.h>

//EEE_IMGPROC defines
#define EEE_IMGPROC_MSG_START ('N'<<16 | 'E'<<8 | 'W')

//offsets
#define EEE_IMGPROC_STATUS 0 
#define EEE_IMGPROC_MSG 1
#define EEE_IMGPROC_ID 2
#define EEE_IMGPROC_BBCOL 3

#define EXPOSURE_INIT 0x00ffff
#define EXPOSURE_STEP 0x100
#define GAIN_INIT 0x100
#define GAIN_STEP 0x040
#define DEFAULT_LEVEL 3

#define MIPI_REG_PHYClkCtl		0x0056
#define MIPI_REG_PHYData0Ctl	0x0058
#define MIPI_REG_PHYData1Ctl	0x005A
#define MIPI_REG_PHYData2Ctl	0x005C
#define MIPI_REG_PHYData3Ctl	0x005E
#define MIPI_REG_PHYTimDly		0x0060
#define MIPI_REG_PHYSta			0x0062
#define MIPI_REG_CSIStatus		0x0064
#define MIPI_REG_CSIErrEn		0x0066
#define MIPI_REG_MDLSynErr		0x0068
#define MIPI_REG_FrmErrCnt		0x0080
#define MIPI_REG_MDLErrCnt		0x0090

void mipi_clear_error(void){
	MipiBridgeRegWrite(MIPI_REG_CSIStatus,0x01FF); // clear error
	MipiBridgeRegWrite(MIPI_REG_MDLSynErr,0x0000); // clear error
	MipiBridgeRegWrite(MIPI_REG_FrmErrCnt,0x0000); // clear error
	MipiBridgeRegWrite(MIPI_REG_MDLErrCnt, 0x0000); // clear error

  	MipiBridgeRegWrite(0x0082,0x00);
  	MipiBridgeRegWrite(0x0084,0x00);
  	MipiBridgeRegWrite(0x0086,0x00);
  	MipiBridgeRegWrite(0x0088,0x00);
  	MipiBridgeRegWrite(0x008A,0x00);
  	MipiBridgeRegWrite(0x008C,0x00);
  	MipiBridgeRegWrite(0x008E,0x00);
  	MipiBridgeRegWrite(0x0090,0x00);
}

void mipi_show_error_info(void){

	alt_u16 PHY_status, SCI_status, MDLSynErr, FrmErrCnt, MDLErrCnt;

	PHY_status = MipiBridgeRegRead(MIPI_REG_PHYSta);
	SCI_status = MipiBridgeRegRead(MIPI_REG_CSIStatus);
	MDLSynErr = MipiBridgeRegRead(MIPI_REG_MDLSynErr);
	FrmErrCnt = MipiBridgeRegRead(MIPI_REG_FrmErrCnt);
	MDLErrCnt = MipiBridgeRegRead(MIPI_REG_MDLErrCnt);
	printf("PHY_status=%xh, CSI_status=%xh, MDLSynErr=%xh, FrmErrCnt=%xh, MDLErrCnt=%xh\r\n", PHY_status, SCI_status, MDLSynErr,FrmErrCnt, MDLErrCnt);
}

void mipi_show_error_info_more(void){
    printf("FrmErrCnt = %d\n",MipiBridgeRegRead(0x0080));
    printf("CRCErrCnt = %d\n",MipiBridgeRegRead(0x0082));
    printf("CorErrCnt = %d\n",MipiBridgeRegRead(0x0084));
    printf("HdrErrCnt = %d\n",MipiBridgeRegRead(0x0086));
    printf("EIDErrCnt = %d\n",MipiBridgeRegRead(0x0088));
    printf("CtlErrCnt = %d\n",MipiBridgeRegRead(0x008A));
    printf("SoTErrCnt = %d\n",MipiBridgeRegRead(0x008C));
    printf("SynErrCnt = %d\n",MipiBridgeRegRead(0x008E));
    printf("MDLErrCnt = %d\n",MipiBridgeRegRead(0x0090));
    printf("FIFOSTATUS = %d\n",MipiBridgeRegRead(0x00F8));
    printf("DataType = 0x%04x\n",MipiBridgeRegRead(0x006A));
    printf("CSIPktLen = %d\n",MipiBridgeRegRead(0x006E));
}



bool MIPI_Init(void){
	bool bSuccess;


	bSuccess = oc_i2c_init_ex(I2C_OPENCORES_MIPI_BASE, 50*1000*1000,400*1000); //I2C: 400K
	if (!bSuccess)
		printf("failed to init MIPI- Bridge i2c\r\n");

    usleep(50*1000);
    MipiBridgeInit();

    usleep(500*1000);

//	bSuccess = oc_i2c_init_ex(I2C_OPENCORES_CAMERA_BASE, 50*1000*1000,400*1000); //I2C: 400K
//	if (!bSuccess)
//		printf("failed to init MIPI- Camera i2c\r\n");

    MipiCameraInit();
    MIPI_BIN_LEVEL(DEFAULT_LEVEL);
//    OV8865_FOCUS_Move_to(340);

//    oc_i2c_uninit(I2C_OPENCORES_CAMERA_BASE);  // Release I2C bus , due to two I2C master shared!


 	usleep(1000);


//    oc_i2c_uninit(I2C_OPENCORES_MIPI_BASE);

	return bSuccess;
}

char NAVIGATION(int word, int * top_1, int * top_3, int * bottom_1, int * bottom_3, FILE* ser) {
	int id = word >> 28;
	int left_1, right_1, left_3, right_3;
	int gradient_l, gradient_r;
	char dir;
	dir = '*';

	//to get word[10:0]
	alt_u32 tmp = word << 21;
	int data_1 = tmp >> 21;

	//to get word[26:16]
	alt_u32 tmp1 = word << 5;
	int data_2 = tmp1 >> 21;

	int x_1, x_2;

	if (id == 6){//word 2
		//data1 is right_min
		
		if ((data_1==640) | (data_2==0)){
			dir = 'c';
		}
		if (data_1 != 640){
			x_1 = data_1/8 + 128;
			if (fwrite(&x_1, 1, 1, ser) != 1)
    		   printf("Error writing to UART");
		}
		if (data_2 != 0){
			x_2 = data_2/8 + 128;
			if (fwrite(&x_2, 1, 1, ser) != 1)
    		   printf("Error writing to UART");
		}
		printf("%03x %03x ", x_1, x_2);
		printf("%03x %03x ", data_1, data_2);
	}

	else if (id == 1){//word 3
		if (data_1 > 440){
			dir = 's';
			int encode = 5;
			//printf("%c ", dir);
			if (fwrite(&encode, 1, 1, ser) != 1)
    	 	   printf("Error writing to UART");
		}
		else {
			dir = '1';
		}
		//printf("%c ", dir);
	}

	else if (id == 2){//word 4
		* top_1 = data_2;
		* bottom_1 = data_1;
		if (* bottom_1 > 440) {
			dir = 's';
			//printf("%c ", dir);
		}
		else{
			dir = '2';
		}
		//printf("%c ", dir);
	}

	else if (id == 3){//word 5
		right_1 = data_2;
		left_1 = data_1;
		gradient_l = ((*bottom_1)-(*top_1))*16/(right_1-left_1);
		if ((gradient_l < 0.1*16)) {
			dir = 's';
			//printf("%c ", dir);
		}
		else if ((gradient_l > 0.2*16) & (gradient_l < 0.75*16)) {
			dir = 'r';
			//printf("%c ", dir);
		}
		else if ((gradient_l > 0.74*16) & (gradient_l < 2.32*16)) {
			dir = 'f';
			//printf("%c ", dir);
		}
		else {
			dir = '3';
		}
		printf("%06x ", gradient_l);
	}

	else if (id==4){//word 6
		* top_3 = data_2;
		* bottom_3 = data_1;
		if (* bottom_3 > 440) {
			dir = 's';
			//printf("%c ", dir);
		}
		else {
			dir = '4';
		}
		//printf("%c ", dir);
	}

	else if (id==5){//word 7
		right_3 = data_1;
		left_3 = data_2;
		gradient_r = ((*bottom_3)-(*top_3))*16/(right_3-left_3);
		if ((gradient_r < 0.1*16)) {
			dir = 's';
			//printf("%c ", dir);
		}
		else if ((gradient_r > 0.2*16) & (gradient_r < 0.75*16)) {
			dir = 'l';
			//printf("%c ", dir);
		}
		else if ((gradient_r > 0.74*16) & (gradient_r < 2.32*16)) {
			dir = 'f';
			//printf("%c ", dir);
		}
		else {
			dir = '5';
		}
		//printf("%c ", dir);
		printf("%06x ", gradient_r);
		//printf("/n");
	}
	else if (id==0){//word 1
		int tmp_word = word;
		bool blue_m = 1&tmp_word;
		tmp_word = tmp_word >> 1;
		bool yellow_m = 1&tmp_word;
		tmp_word = tmp_word >> 1;
		bool red_m = 1&tmp_word;
		tmp_word = tmp_word >> 1;
		bool blue_short = 1&tmp_word;
		tmp_word = tmp_word >> 1;
		bool blue_long = 1&tmp_word;
		tmp_word = tmp_word >> 1;
		bool yellow_beacon = 1&tmp_word;
		tmp_word = tmp_word >> 1;
		bool red_beacon = 1&tmp_word;

		if (blue_m) {
			int encode = 12;
			if (fwrite(&encode, 1, 1, ser) != 1)
    		   printf("Error writing to UART");
		}
		if (yellow_m) {
			int encode = 11;
			if (fwrite(&encode, 1, 1, ser) != 1)
    		   printf("Error writing to UART");
		}
		if (red_m) {
			int encode = 10;
			if (fwrite(&encode, 1, 1, ser) != 1)
    		   printf("Error writing to UART");
		}
		if (blue_short) {
			int encode = 8;
			if (fwrite(&encode, 1, 1, ser) != 1)
    		   printf("Error writing to UART");
		}
		if (blue_long) {
			int encode = 7;
			if (fwrite(&encode, 1, 1, ser) != 1)
    		   printf("Error writing to UART");
		}
		if (yellow_beacon) {
			int encode = 6;
			if (fwrite(&encode, 1, 1, ser) != 1)
    		   printf("Error writing to UART");
		}
		if (red_beacon) {
			int encode = 4;
			if (fwrite(&encode, 1, 1, ser) != 1)
    		   printf("Error writing to UART");
		}
	}
	return dir;
}




int main()
{

	fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

  printf("DE10-LITE D8M VGA Demo\n");
  printf("Imperial College EEE2 Project version\n");
  IOWR(MIPI_PWDN_N_BASE, 0x00, 0x00);
  IOWR(MIPI_RESET_N_BASE, 0x00, 0x00);

  usleep(2000);
  IOWR(MIPI_PWDN_N_BASE, 0x00, 0xFF);
  usleep(2000);
  IOWR(MIPI_RESET_N_BASE, 0x00, 0xFF);

  printf("Image Processor ID: %x\n",IORD(0x42000,EEE_IMGPROC_ID));
  //printf("Image Processor ID: %x\n",IORD(EEE_IMGPROC_0_BASE,EEE_IMGPROC_ID)); //Don't know why this doesn't work - definition is in system.h in BSP


  usleep(2000);


  // MIPI Init
   if (!MIPI_Init()){
	  printf("MIPI_Init Init failed!\r\n");
  }else{
	  printf("MIPI_Init Init successfully!\r\n");
  }

//   while(1){
 	    mipi_clear_error();
	 	usleep(50*1000);
 	    mipi_clear_error();
	 	usleep(1000*1000);
	    mipi_show_error_info();
//	    mipi_show_error_info_more();
	    printf("\n");
//   }


#if 0  // focus sweep
	    printf("\nFocus sweep\n");
 	 	alt_u16 ii= 350;
 	    alt_u8  dir = 0;
 	 	while(1){
 	 		if(ii< 50) dir = 1;
 	 		else if (ii> 1000) dir =0;

 	 		if(dir) ii += 20;
 	 		else    ii -= 20;

 	    	printf("%d\n",ii);
 	     OV8865_FOCUS_Move_to(ii);
 	     usleep(50*1000);
 	    }
#endif






    //////////////////////////////////////////////////////////
        alt_u16 bin_level = DEFAULT_LEVEL;
        alt_u8  manual_focus_step = 10;
        alt_u16  current_focus = 300;
    	int boundingBoxColour = 0;
    	alt_u32 exposureTime = EXPOSURE_INIT;
    	alt_u16 gain = GAIN_INIT;
		int top_1, top_3, bottom_1, bottom_3;
		top_1 = 0;
		top_3 = 0;
		bottom_1 = 0;
		bottom_3 = 0;
		char word_1, word_2, word_3, word_4, word_5, word_6, word_7;

        OV8865SetExposure(exposureTime);
        OV8865SetGain(gain);
        Focus_Init();

        FILE* ser = fopen("/dev/uart_0", "rb+");
        if(ser){
        	printf("Opened UART\n");
        } else {
        	printf("Failed to open UART\n");
        	while (1);
        }

  while(1){

       // touch KEY0 to trigger Auto focus
	   if((IORD(KEY_BASE,0)&0x03) == 0x02){

    	   current_focus = Focus_Window(320,240);
       }
	   // touch KEY1 to ZOOM
	         if((IORD(KEY_BASE,0)&0x03) == 0x01){
	      	   if(bin_level == 3 )bin_level = 1;
	      	   else bin_level ++;
	      	   printf("set bin level to %d\n",bin_level);
	      	   MIPI_BIN_LEVEL(bin_level);
	      	 	usleep(500000);

	         }


	#if 0
       if((IORD(KEY_BASE,0)&0x0F) == 0x0E){

    	   current_focus = Focus_Window(320,240);
       }

       // touch KEY1 to trigger Manual focus  - step
       if((IORD(KEY_BASE,0)&0x0F) == 0x0D){

    	   if(current_focus > manual_focus_step) current_focus -= manual_focus_step;
    	   else current_focus = 0;
    	   OV8865_FOCUS_Move_to(current_focus);

       }

       // touch KEY2 to trigger Manual focus  + step
       if((IORD(KEY_BASE,0)&0x0F) == 0x0B){
    	   current_focus += manual_focus_step;
    	   if(current_focus >1023) current_focus = 1023;
    	   OV8865_FOCUS_Move_to(current_focus);
       }

       // touch KEY3 to ZOOM
       if((IORD(KEY_BASE,0)&0x0F) == 0x07){
    	   if(bin_level == 3 )bin_level = 1;
    	   else bin_level ++;
    	   printf("set bin level to %d\n",bin_level);
    	   MIPI_BIN_LEVEL(bin_level);
    	 	usleep(500000);

       }
	#endif

       //Read messages from the image processor and print them on the terminal
	   
       while ((IORD(0x42000,EEE_IMGPROC_STATUS)>>8) & 0xff) { 	//Find out if there are words to read
		   int word;
    	   while(1){
			word = IORD(0x42000,EEE_IMGPROC_MSG);
			if ((word>>28)==0) {
		        word_1 = NAVIGATION(word, & top_1, & top_3, & bottom_1, & bottom_3, ser);
				printf("%c ", word_1);
				break;
			}
		   }

		   while(1){
			if ((IORD(0x42000,EEE_IMGPROC_STATUS)>>8) & 0xff) {
				word = IORD(0x42000,EEE_IMGPROC_MSG); 			//Get next word from message buffer
		        word_2 = NAVIGATION(word, & top_1, & top_3, & bottom_1, & bottom_3, ser);
				printf("%c ", word_2);
				break;
			}
		   }

		   while(1){
			if ((IORD(0x42000,EEE_IMGPROC_STATUS)>>8) & 0xff) {
				word = IORD(0x42000,EEE_IMGPROC_MSG); 			//Get next word from message buffer
		        word_3 = NAVIGATION(word, & top_1, & top_3, & bottom_1, & bottom_3, ser);
				printf("%c ", word_3);
				break;
			}
		   }

		   while(1){
			if ((IORD(0x42000,EEE_IMGPROC_STATUS)>>8) & 0xff) {
				word = IORD(0x42000,EEE_IMGPROC_MSG); 			//Get next word from message buffer
		        word_4 = NAVIGATION(word, & top_1, & top_3, & bottom_1, & bottom_3, ser);
				printf("%c ", word_4);
				break;
			}
		   }

		   while(1){
			if ((IORD(0x42000,EEE_IMGPROC_STATUS)>>8) & 0xff) {
				word = IORD(0x42000,EEE_IMGPROC_MSG); 			//Get next word from message buffer
		        word_5 = NAVIGATION(word, & top_1, & top_3, & bottom_1, & bottom_3, ser);
				printf("%c ", word_5);
				break;
			}
		   }

		   while(1){
			if ((IORD(0x42000,EEE_IMGPROC_STATUS)>>8) & 0xff) {
				word = IORD(0x42000,EEE_IMGPROC_MSG); 			//Get next word from message buffer
		        word_6 = NAVIGATION(word, & top_1, & top_3, & bottom_1, & bottom_3, ser);
				printf("%c ", word_6);
				break;
			}
		   }

		   while(1){
			if ((IORD(0x42000,EEE_IMGPROC_STATUS)>>8) & 0xff) {
				word = IORD(0x42000,EEE_IMGPROC_MSG); 			//Get next word from message buffer
		        word_7 = NAVIGATION(word, & top_1, & top_3, & bottom_1, & bottom_3, ser);
				printf("%c \n", word_7);
				break;
			}
		   }

		   int encode;

		   if (word_2 == 'c') {//check node
			encode = 9;
			fwrite(&encode, 1, 1, ser);
		   }

		   if (word_3 == 's' & word_4 == 's' & word_6 == 's'){//stop moving forward
			encode = 5;
			//printf("%c ", word_3);
			if (fwrite(&encode, 1, 1, ser) != 1)
    	 	   printf("Error writing to UART");
		   }
		//    else{
		// 	encode = 1;
		// 	//printf("%c ", word_3);
		// 	if (fwrite(&encode, 1, 1, ser) != 1)
    	//  	   printf("Error writing to UART");
		//    }

		   if (word_5 == 'f' & word_7 == 'f'){//can move forward
			encode = 1;
			printf("%03x \n", encode);
			if (fwrite(&encode, 1, 1, ser) != 1)
    		   printf("Error writing to UART");
		   }

		   if (word_3 != 's' & word_4 == 's'){//corect direction to the right
			encode = 3;
			fwrite(&encode, 1, 1, ser);
		   }

		   if (word_3 != 's' & word_6 == 's'){//correct direction to the left
			encode = 2;
			fwrite(&encode, 1, 1, ser);
		   }

    	   
       }

       //Update the bounding box colour
       //boundingBoxColour = ((boundingBoxColour + 1) & 0xff);
       //IOWR(0x42000, EEE_IMGPROC_BBCOL, (boundingBoxColour << 8) | (0xff - boundingBoxColour));

       //Process input commands
       int in = getchar();
       switch (in) {
       	   case 'e': {
       		   exposureTime += EXPOSURE_STEP;
       		   OV8865SetExposure(exposureTime);
       		   printf("\nExposure = %x ", exposureTime);
       	   	   break;}
       	   case 'd': {
       		   exposureTime -= EXPOSURE_STEP;
       		   OV8865SetExposure(exposureTime);
       		   printf("\nExposure = %x ", exposureTime);
       	   	   break;}
       	   case 't': {
       		   gain += GAIN_STEP;
       		   OV8865SetGain(gain);
       		   printf("\nGain = %x ", gain);
       	   	   break;}
       	   case 'g': {
       		   gain -= GAIN_STEP;
       		   OV8865SetGain(gain);
       		   printf("\nGain = %x ", gain);
       	   	   break;}
       	   case 'r': {
        	   current_focus += manual_focus_step;
        	   if(current_focus >1023) current_focus = 1023;
        	   OV8865_FOCUS_Move_to(current_focus);
        	   printf("\nFocus = %x ",current_focus);
       	   	   break;}
       	   case 'f': {
        	   if(current_focus > manual_focus_step) current_focus -= manual_focus_step;
        	   OV8865_FOCUS_Move_to(current_focus);
        	   printf("\nFocus = %x ",current_focus);
       	   	   break;}
       }


	   //Main loop delay
	   usleep(10000);

   };
  return 0;
}
