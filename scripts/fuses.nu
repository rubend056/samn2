##################### FUSES ATMEGA328P/B #######################
###############  BE EXTREMELY CAREFUL WITH THIS ################
###############  0 means ON, 1 means OFF        ################
###############  Definitely use the atmega datasheet ###########
######## EXTENDED
#	7			None				 			1
#	6			None							1
#	5			None							1
#	4			None					 		1
#	3			Disable clock failure detection	0 <- atmega328P didn't have this feature (None), and it was by default '1'
#	2			BODL2							1
#	1			BODL1							1
#	0			BODL0							1
######## HIGH
#	7			External Reset Disable 			1 (NEVER TURN ON)
#	6			Debug Wire Enable				1 
#	5			Serial, Data Downloading (SPI)	0
#	4			Watchdog Timer Always ON 		1
#	3			Enable EEPROM Preserve			1
#	2			BootSize1						0 # datashset wrong
#	1			BootSize0						0 # datasheet wrong
#	0			Select Reset Vector				1 # datasheet wrong
######## LOW
#	Bit			Description						Default
#	7			Divide clock by 8	 			0
#	6			Clock Output					1
#	5			Startup Time 1					1
#	4			Startup Time 0					0
#	3			Clock Source 3					0
#	2			Clock Source 2					0
#	1			Clock Source 1					1
#	0			Clock Source 0					0


def bit_on [v,bit] {
    $v | bits and (0b[1] | bits shl $bit| bits not)
}
def bit_off [v,bit] {
    $v | bits or (0b[1] | bits shl $bit)
}
def fuse_config_samn_v9 [] {
    # atmega328pb defaults
    mut ext = 0b[1111 0111]
    mut high = 0b[1101 1001]
    mut low = 0b[0110 0010]

    # Here we configure the settings we want
    
    # Enable brown out detector for 2.7V
    $ext = (bit_on $ext 1)
    # Don't divide clock by 8 internally
    $low = (bit_off $low 7)

    [$ext $high $low]
}
def fuse_config_samn_v8 [] {
    # atmega328p defaults
    mut ext = 0b[1111 1111]
    mut high = 0b[1101 1001]
    mut low = 0b[0110 0010]

    # Here we configure the settings we want
    
    # Enable brown out detector for 1.7V
    $ext = (bit_on $ext 0)
    # Don't divide clock by 8 internally
    $low = (bit_off $low 7)

    [$ext $high $low]
}
export def fuse_samn_v9 [] {
    let fuses = fuse_config_samn_v9
    let args = [
        "-p" "m328pb"
        "-c" "usbasp"
        "-U" $"efuse:w:0x($fuses | get 0 | encode hex):m"
        "-U" $"hfuse:w:0x($fuses | get 1 | encode hex):m"
        "-U" $"lfuse:w:0x($fuses | get 2 | encode hex):m"
    ]

    avrdude ...$args
}
export def fuse_samn_v8 [] {
    let fuses = fuse_config_samn_v8
    let args = [
        "-p" "m328p"
        "-c" "usbasp"
        "-U" $"efuse:w:0x($fuses | get 0 | encode hex):m"
        "-U" $"hfuse:w:0x($fuses | get 1 | encode hex):m"
        "-U" $"lfuse:w:0x($fuses | get 2 | encode hex):m"
    ]

    avrdude ...$args
}
