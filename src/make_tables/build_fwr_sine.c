#include <stdio.h>
#include <math.h>
void main()
{
	int i;
	for(i=0; i<512; i++)
	{
		if(i%16 == 0)
			printf("\n");
		if(i<256)
			printf("%4d,", (int) (600.0*sin( 2.0*M_PI*((double)i)/((double)512))  ) );
		else
			printf("%4d,", (int) (-600.0*sin( 2.0*M_PI*((double)i)/((double)512))  ) );		
	}
}
