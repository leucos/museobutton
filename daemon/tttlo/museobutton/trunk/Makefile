.PHONY: doc daemon firmware

all: doc daemon firmware

daemon:
	@(cd daemon && make)

firmware:
	@(cd firmware && make)

doc:
	@doxygen doxyfile

clean:
	@rm -rf doc/
	@rm -f *~
	@(cd daemon && make clean)
	@(cd firmware && make clean)

