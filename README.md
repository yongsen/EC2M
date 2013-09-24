EC2M
====

Multipath TCP (<a href="http://mptcp.info.ucl.ac.be/" target="_blank">UCL Homepage</a> or <a href="http://datatracker.ietf.org/wg/mptcp/charter/" target="_blank">IETF Homepage</a>) enables concurrent wireless links for mobile terminals, but it needs modifications to leverage path diversity and improve energy efficiency. It requires cross-layer design to overcome the mismatch between transport layer and wireless links, and to allocate congestion windows for WiFi and 3G subflows. Some detailed information will be updated lately.

This repository includes the source code of MPTCP on NS3. It can build concurrent WiFi and LTE connections by MPTCP. Currently, it can only simulate traditional congestion control algorithms. Other congestion control methods and cross-layer networking strategies will be included.
