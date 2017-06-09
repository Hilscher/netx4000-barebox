#
# Arch-specific CryptoAPI modules.
#

obj-$(CONFIG_DIGEST_SHA1_ARM) += sha1-arm.o
obj-$(CONFIG_DIGEST_SHA256_ARM) += sha256-arm.o
obj-$(CONFIG_DIGEST_SHA512_ARM) += sha512-arm.o

sha1-arm-y	:= sha1-armv4-large.o sha1_glue.o
ifneq ($(CONFIG_FPUTYPE),"neon")
sha256-arm-y	:= sha256-core.o sha256_glue.o
sha512-arm-y	:= sha512-core.o sha512_glue.o
else
sha256-arm-y    := sha256-core.o sha256_neon_glue.o
sha512-arm-y	:= sha512-core.o sha512_neon_glue.o
endif

quiet_cmd_perl = PERL    $@
      cmd_perl = $(PERL) $(<) > $(@)

$(src)/sha256-core.S_shipped: $(src)/sha256-armv4.pl
	$(call cmd,perl)

$(src)/sha512-core.S_shipped: $(src)/sha512-armv4.pl
	$(call cmd,perl)

.PRECIOUS: $(obj)/sha256-core.S
.PRECIOUS: $(obj)/sha512-core.S
