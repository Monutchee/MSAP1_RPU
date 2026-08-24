#ifndef MSAP1_R5C1_HLS_COMPAT_AP_AXI_SDATA_H
#define MSAP1_R5C1_HLS_COMPAT_AP_AXI_SDATA_H

#include <ap_int.h>

/**
 * Allocation-free subset of ap_axiu used by the shared record serializer.
 *
 * User, ID, and destination widths are zero in the MSAP1 record stream.  The
 * full Vitis header adds host exception support for unsupported side-channel
 * combinations, which is inappropriate in freestanding R5 firmware.
 */
template <int DataWidth, int UserWidth, int IdWidth, int DestWidth>
struct ap_axiu final {
	static_assert(DataWidth > 0 && (DataWidth % 8) == 0);
	static_assert(UserWidth == 0 && IdWidth == 0 && DestWidth == 0,
		"R5 compatibility ap_axiu supports the MSAP1 no-metadata stream only");

	ap_uint<DataWidth> data{};
	ap_uint<DataWidth / 8> keep{};
	ap_uint<DataWidth / 8> strb{};
	ap_uint<1> last{};
};

#endif // MSAP1_R5C1_HLS_COMPAT_AP_AXI_SDATA_H
