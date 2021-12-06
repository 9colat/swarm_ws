#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>


int main ()
{
  int serial_port ;
  char dat;
  if ((serial_port = serialOpen ("/dev/ttyS0", 9600)) < 0)	/* open serial port */
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

  if (wiringPiSetup () == -1)					/* initializes wiringPi setup */
  {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    return 1 ;
  }

  while(1){

	if(serialDataAvail (serial_port) )
	{
		dat = serialGetchar (serial_port);		/* receive character serially*/
		printf ("%c", dat) ;
		fflush (stdout) ;
		serialPutchar(serial_port, dat);		/* transmit character serially on port */
		  }
	}

}



/*int serialOpen (char *ttyS0, int 9600)
int serialOpen (char *ttyAMA0, int 9600)

int main (void)
{
  wiringPiSetup();
  pinMode (15, OUTPUT);
  pinMode (16, INPUT);

  for (;;)
  {
    digitalWrite (15, HIGH) ; delay(500);
    digitalWrite (16,  LOW) ; delay(500);
  }
  return 0 ;
}
*/
