<<<<<<< HEAD
obj-$(CONFIG_CRYPTO_DEV_ASPEED) += aspeed_crypto.o aspeed_g6_crypto.o
aspeed_crypto-objs := aspeed-hace.o \
		      aspeed-hace-crypto.o \
		      aspeed-hace-hash.o

aspeed_g6_crypto-objs := aspeed-acry.o \
		         aspeed-acry-rsa.o

=======
hace-hash-$(CONFIG_CRYPTO_DEV_ASPEED_HACE_HASH) := aspeed-hace-hash.o
hace-crypto-$(CONFIG_CRYPTO_DEV_ASPEED_HACE_CRYPTO) := aspeed-hace-crypto.o

obj-$(CONFIG_CRYPTO_DEV_ASPEED) += aspeed_crypto.o
aspeed_crypto-objs := aspeed-hace.o	\
		      $(hace-hash-y)	\
		      $(hace-crypto-y)
>>>>>>> origin/linux_6.1.15_upstream
