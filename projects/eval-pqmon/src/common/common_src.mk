INCS +=		$(PROJECT)/src/common/afe_config.h 	\
		$(PROJECT)/src/common/afe_calibration.h	\
		$(PROJECT)/src/common/pqlib_example.h	\
		$(PROJECT)/src/common/pqlib_afe.h	\
		$(PROJECT)/src/common/pqlib_convert.h	\
		$(PROJECT)/src/common/status.h 		\
		$(PROJECT)/src/common/common_data.h	\
		$(PROJECT)/src/common/iio_pqm.h		\
		$(PROJECT)/src/common/flash_storage.h

SRCS +=		$(PROJECT)/src/common/afe_config.c 	\
		$(PROJECT)/src/common/afe_calibration.c	\
		$(PROJECT)/src/common/pqlib_example.c	\
		$(PROJECT)/src/common/pqlib_afe.c	\
		$(PROJECT)/src/common/pqlib_convert.c	\
		$(PROJECT)/src/common/common_data.c	\
		$(PROJECT)/src/common/iio_pqm.c		\
		$(PROJECT)/src/common/flash_storage.c

ifeq ($(TIME_SYNC), y)
INCS +=		$(PROJECT)/src/common/gnss_utils.h	\
		$(PROJECT)/src/interrupt/interrupt.h	\
		$(PROJECT)/src/common/pps_utils.h	\
		$(PROJECT)/src/common/rtc_utils.h

SRCS +=		$(PROJECT)/src/common/gnss_utils.c	\
		$(PROJECT)/src/interrupt/interrupt.c	\
		$(PROJECT)/src/common/pps_utils.c	\
		$(PROJECT)/src/common/rtc_utils.c
endif
