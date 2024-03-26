export def build [board, bin] {
    let root = $env.PWD
    enter $"boards/($board)"
        cargo build --release -Zunstable-options --out-dir $"($root)/out/($board)" --bin $bin
}
def get_avrdude_mega [board] {
    enter $"boards/($board)"
        $"m(open .cargo/config.toml | get build.target | parse --regex 'atmega(?P<board>.*)\.json' | get board.0)"
}
export def main [board, bin] {
    build $board $bin
    avrdude -p (get_avrdude_mega $board) -c usbasp -U $"flash:w:out/($board)/($bin).elf"
}