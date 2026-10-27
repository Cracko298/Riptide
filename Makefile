CC ?= cc
AR ?= ar
BUILD ?= build
CFLAGS ?= -O2 -g
CPPFLAGS += -Iinclude -Ithird_party/miniz -DMINIZ_NO_ARCHIVE_APIS -DMINIZ_NO_ZLIB_COMPATIBLE_NAMES
WARN = -Wall -Wextra -Wpedantic
THREAD_FLAGS ?= -pthread
LDLIBS = -lm $(THREAD_FLAGS)
CORE = core vehicle physics world route render raster jobs image gpu_stream
SRC = $(addprefix src/,$(addsuffix .c,$(CORE))) third_party/miniz/miniz.c third_party/miniz/miniz_tdef.c third_party/miniz/miniz_tinfl.c
OBJ = $(patsubst %.c,$(BUILD)/%.o,$(SRC))
DEPS = $(OBJ:.o=.d)
.PHONY: all test sanitize clean preview windows
all: $(BUILD)/libriptide.a $(BUILD)/riptide-sandbox
$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(WARN) -std=c11 $(THREAD_FLAGS) -MMD -MP -c $< -o $@
$(BUILD)/libriptide.a: $(OBJ)
	$(AR) rcs $@ $^
$(BUILD)/riptide-sandbox: examples/headless.c examples/sandbox.c examples/sandbox.h $(BUILD)/libriptide.a
	$(CC) $(CPPFLAGS) $(CFLAGS) $(WARN) -std=c11 examples/headless.c examples/sandbox.c $(BUILD)/libriptide.a $(LDLIBS) -o $@
$(BUILD)/riptide-tests: tests/test_engine.c $(BUILD)/libriptide.a
	$(CC) $(CPPFLAGS) $(CFLAGS) $(WARN) -std=c11 $< $(BUILD)/libriptide.a $(LDLIBS) -o $@
test: $(BUILD)/riptide-tests
	./$(BUILD)/riptide-tests
preview: $(BUILD)/riptide-sandbox
	./$(BUILD)/riptide-sandbox $(BUILD)/sandbox.png
sanitize:
	$(MAKE) BUILD=build-sanitize CFLAGS='-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer' test preview
windows: $(BUILD)/libriptide.a
	$(CC) $(CPPFLAGS) $(CFLAGS) $(WARN) -std=c11 -mwindows examples/windows.c examples/sandbox.c $(BUILD)/libriptide.a -lgdi32 -luser32 -lm -o $(BUILD)/riptide-sandbox.exe
clean:
	rm -rf $(BUILD)
-include $(DEPS)
