################################################################################
#
# n7d
#
################################################################################

N7D_VERSION = 0.1
N7D_SITE = $(BR2_EXTERNAL_N7D_PATH)
N7D_SITE_METHOD = local
N7D_LICNSE = GPL-2.0
N7D_LICENSE_FILES = LICENSE

$(eval $(kernel-module))
$(eval $(generic-package))
