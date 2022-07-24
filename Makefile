CC := clang
CFLAGS := -std=c99 -g -O2 -Wall -Wextra -pipe -Iinclude/
TARGET := bin/flameboy

CFILES := $(shell find . -name "*.c")
OFILES := $(addprefix bin/,$(CFILES:.c=.o))
HEADERDEPS := $(OFILES:.o=.d)

$(TARGET): $(OFILES)
	$(CC) $^ -o $@

-include $(HEADERDEPS)
bin/%.o: %.c
	mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: run
run: $(TARGET)
	$(TARGET)

.PHONY: clean
clean:
	rm -r bin/
