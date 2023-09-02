# https://triple-four.hatenablog.com/entry/20210623/1624458051#-Wall-%E3%81%A8--Wextra--%E3%81%84%E3%82%8D%E3%81%84%E3%82%8D%E3%81%AA%E8%AD%A6%E5%91%8A%E3%82%92%E3%81%BE%E3%81%A8%E3%82%81%E3%81%A6%E6%9C%89%E5%8A%B9%E5%8C%96
# https://img.atcoder.jp/file/language-update/language-list.html
# aliasコマンド で .bashrc に定義もしている
# alias g+='g++ -std=gnu++2a -O2 -g -mtune=native -march=native -fconstexpr-depth=2147483647 -fconstexpr-loop-limit=2147483647 -fconstexpr-ops-limit=2147483647 -Wall -Wextra -Wfloat-equal -ftrapv -fstack-protector-all -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover -DMARC_LOCAL -o a.out'
# alias ainout='./a.out < input.txt > output.txt'


CC = g++
VERSION = -std=gnu++2a
WARNINGS = -Wall -Wextra -Wfloat-equal
FLAGS = -ftrapv -fstack-protector-all -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover
CFLAGS = -O2 -g -mtune=native -march=native -fconstexpr-depth=2147483647 -fconstexpr-loop-limit=2147483647 -fconstexpr-ops-limit=2147483647
DEFINE = -DMARC_LOCAL
INCDIR = -I .
TARGET = a.out


main: main.cpp
	$(CC) $(VERSION) $(WARNINGS) $(DEFINE) -o $(TARGET) $<

main2: main.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -o $(TARGET) $<

mainat: main.cpp
	$(CC) $(VERSION) $(WARNINGS) $(DEFINE) -DATCODER $(INCDIR) -o $(TARGET) $<

mainat2: main.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -DATCODER $(INCDIR) -o $(TARGET) $<

test: test.cpp
	$(CC) $(VERSION) $(WARNINGS) $(DEFINE) -o $(TARGET) $<

test2: test.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -o $(TARGET) $<

testat: test.cpp
	$(CC) $(VERSION) $(WARNINGS) $(DEFINE) -DATCODER $(INCDIR) -o $(TARGET) $<

testat2: test.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -DATCODER $(INCDIR) -o $(TARGET) $<

naive: naive.cpp
	$(CC) $(VERSION) $(WARNINGS) $(DEFINE) -o $(TARGET) $<

naive2: naive.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -o $(TARGET) $<

naiveat: naive.cpp
	$(CC) $(VERSION) $(WARNINGS) $(DEFINE) -DATCODER $(INCDIR) -o $(TARGET) $<

naiveat2: naive.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -DATCODER $(INCDIR) -o $(TARGET) $<

libcheck: libcheck.cpp
	$(CC) $(VERSION) $(CFLAGS) $(WARNINGS) $(FLAGS) $(DEFINE) -o $(TARGET) $<


.PHONY: clean
clean:
	rm -f $(TARGET) *.o