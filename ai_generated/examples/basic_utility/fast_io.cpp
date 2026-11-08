#include <cstdio>
#include <stdexcept>
#include "basic_utility/fast_io.hpp"
int main() {
    std::FILE* file = std::tmpfile();
    if (!file) throw std::runtime_error("tmpfile failed");
    {
        cp::FastPrinter writer(file);
        writer.writeln(-42, "hello", 1.25);
        writer.flush();
    }
    std::rewind(file);
    {
        cp::FastScanner reader(file);
        __int128_t number; std::string word; double real;
        if (reader.read(number, word, real)) {
            cp::FastPrinter out;
            out.writeln(number, word);
            out.write_fixed(real, 2); out.writeln(); out.flush();
        }
    }
    std::fclose(file);
}
