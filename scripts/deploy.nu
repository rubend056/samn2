export def build [board, bin] {
    let root = $env.PWD
    let args = [
        #"-Zlocation-detail=none",
        "-Zunstable-options",
        "-Zbuild-std-features=panic_immediate_abort",
        "--out-dir",$"($root)/out/($board)"
    ]
    enter $"boards/($board)"
        cargo build ...$args --release --bin $bin
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