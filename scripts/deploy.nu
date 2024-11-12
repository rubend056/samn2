export def psize [board, bin] {
    let root = $env.PWD
    let out = $"($root)/out/($board)";
    avr-size -C --mcu=atmega328p $"($out)/($bin).elf"
}
export def build [board, bin] {
    let root = $env.PWD
    let out = $"($root)/out/($board)";
    let args = [
        #"-Zlocation-detail=none",
        "-Zunstable-options",
        #"-Zbuild-std-features=panic_immediate_abort",
        "--out-dir", $out
    ]
    enter $"boards/($board)"
        cargo build ...$args --release --bin $bin
    dexit
    psize $board $bin
}
def get_avrdude_mega [board] {
    enter $"boards/($board)"
        $"m(open .cargo/config.toml | get build.target | parse --regex 'atmega(?P<board>.*)\.json' | get board.0)"
}
export def main [board, bin] {
    build $board $bin
    avrdude -p (get_avrdude_mega $board) -c usbasp -U $"flash:w:out/($board)/($bin).elf"
}
export def run [board, bin] {
    enter $"boards/($board)"
        cargo run --release --bin $bin
}

export def burn_new_id [] {
    if not ("ids.json" | path exists) {
        # [] | save ids.json
        print "We couldn't find ids.json"
        print "We could make a new file"
        print "But then you wouldn't know we couldn't find it"
        print "So we're erring on the side of caution and exiting"
        exit
    }
    mut ids = open ids.json
    
    # Generate a new id
    mut id = (random int 0..4294967295)
    while $id in $ids {
        $id = (random int 0..4294967295)
    }
    
    $ids = ($ids | append $id)
    # Int -> binary (8 bytes) -> get first 2 bytes -> reverse so the hex encoding is big endian 
    # (as it should, so avrdude can then turn it to little endian back again)
    let hex = ($id | into binary | bytes at 0..4 | bytes reverse | encode hex)
    avrdude -p m328pb -c usbasp -U $"eeprom:w:0x($hex):m"

    $ids | save -f ids.json
}