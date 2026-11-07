Copied from AlgoWorkspace-26/src/TreeAction/ares_usb/ARES_bulk_library on 2026-11-07.
Local change: USB write timeout 1000 -> 10 ms. The bridge does not retain failed commands or reconnect automatically.
Protocol: original ROS bridge swaps 0x0111 to 0x1101 before send_sync (which uses htons).
Preserve this wire convention. Incoming IDs are exposed as the original library returns them.
Upstream package declares Apache License 2.0; upstream source headers are preserved.
