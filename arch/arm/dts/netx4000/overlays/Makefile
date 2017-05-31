# Build dt-overlays for Hilscher netX4000 based platforms

%.dtbo: %.dtso
	$(CPP) $(dtc_cpp_flags) -x assembler-with-cpp -o $<.tmp $< ; \
	$(objtree)/scripts/dtc/dtc -O dtb -o $@ ${DTC_FLAGS} $<.tmp && \
	rm $<.tmp

src = $(shell find arch/arm/dts/netx4000/overlays -name *.dtso -exec basename {} \;)
dtbo-$(CONFIG_ARCH_NETX4000) += ${src:.dtso=.dtbo}

always		:= $(dtbo-y)
clean-files	:= *.dtbo