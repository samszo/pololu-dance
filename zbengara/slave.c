#include <pololu/3pi.h>
#include <ctype.h>

// PID constants
unsigned int pid_enabled = 0;
unsigned char max_speed = 255;
unsigned char p_num = 0;
unsigned char p_den = 0;
unsigned char d_num = 0;
unsigned char d_den = 0;
unsigned int last_proportional = 0;
unsigned int sensors[5];

char buffer[100];

// This routine will be called repeatedly to keep the PID algorithm running
void pid_check()
{
	if(!pid_enabled)
		return;
	
	// Do nothing if the denominator of any constant is zero.
	if(p_den == 0 || d_den == 0)
	{
		set_motors(0,0);
		return;
	}	

	// Read the line position.
	unsigned int position = read_line(sensors, IR_EMITTERS_ON);

	// The "proportional" term should be 0 when we are on the line.
	int proportional = ((int)position) - 2000;

	// Compute the derivative (change) of the position.
	int derivative = proportional - last_proportional;

	// Remember the last position.
	last_proportional = proportional;

	// Compute the difference between the two motor power settings,
	// m1 - m2.  If this is a positive number the robot will turn
	// to the right.  If it is a negative number, the robot will
	// turn to the left, and the magnitude of the number determines
	// the sharpness of the turn.
	int power_difference = proportional*p_num/p_den + derivative*d_num/d_den;

	// Compute the actual motor settings.  We never set either motor
	// to a negative value.
	if(power_difference > max_speed)
		power_difference = max_speed;
	if(power_difference < -max_speed)
		power_difference = -max_speed;

	if(power_difference < 0)
		set_motors(max_speed+power_difference, max_speed);
	else
		set_motors(max_speed, max_speed-power_difference);
}

// A pointer to where we are reading from.
unsigned char read_index = 0;

// Waits for the next byte and returns it.  Runs play_check to keep
// the music playing and calls pid_check() to keep following the line.
char read_next_byte()
{
	while(serial_get_received_bytes() == read_index)
	{
		play_check();

		// pid_check takes some time; only run it if we don't have more bytes to process
		if(serial_get_received_bytes() == read_index)
		  pid_check();
		
	}
	char ret = buffer[read_index];
	read_index ++;
	if(read_index >= 100)
		read_index = 0;
	return ret;
}

// Backs up by one byte in the ring buffer.
void previous_byte()
{
	read_index --;
	if(read_index == 255)
		read_index = 99;
}
int vitessea=30;
int vitesset=20;
int ttmp=15;
int main(void) {
  
  pololu_3pi_init(2000);  
  play_mode(PLAY_CHECK);
  clear();
print("Hello!");
  play("L16 ceg>c");
  // start receiving data at 9600 baud
  serial_set_baud_rate(9600);
  serial_receive_ring(buffer, 100);
 
  while(1) {
    // wait for a command
    char command = read_next_byte();

    switch(command) {
	case 'A':
	set_motors(vitessea,vitessea);

delay_ms(ttmp);
set_motors(0,0);
	  break;
	
case 'D':
set_motors(vitesset,-1*vitesset);
delay_ms(ttmp);
set_motors(0,0);
	  break;
case 'G':
set_motors(-1*vitesset,vitesset);
delay_ms(ttmp);
set_motors(0,0);
	  break;
case 'M':
 play("! O5 L16 agafaea dac+adaea fa<aa<bac#a dac#adaea f"
  "O6 dcd<b-d<ad<g d<f+d<gd<ad<b- d<dd<ed<f+d<g d<f+d<gd<ad"
  "L8 MS <b-d<b-d MLe-<ge-<g MSc<ac<a ML d<fd<f O5 MS b-gb-g"
  "ML >c#e>c#e MS afaf ML gc#gc# MS fdfd ML e<b-e<b-"
  "O6 L16ragafaea dac#adaea fa<aa<bac#a dac#adaea faeadaca"
  "<b-acadg<b-g egdgcg<b-g <ag<b-gcf<af dfcf<b-f<af"
  "<gf<af<b-e<ge c#e<b-e<ae<ge <fe<ge<ad<fd"
  "O5 e>ee>ef>df>d b->c#b->c#a>df>d e>ee>ef>df>d"
  "e>d>c#>db>d>c#b >c#agaegfe f O6 dc#dfdc#<b c#4");


default:
set_motors(0,0);
      break;
    }
  }
}
