# MSAP1 RPU firmware

This repository contains the FreeRTOS applications for both ZynqMP R5 cores.
The workspace `make_RPU.sh` flow builds the Vitis platform directly from the
product XSA and generates one `openamp_contract.h` for each core.

The OpenAMP interface deliberately has two sources of truth:

- the XSA-generated `xparameters.h` owns AXI peripheral addresses and
  interrupt assignments;
- the manifest-owned `openamp-contract.json` owns RPMsg shared memory and
  APU/R5 mailbox policy.

The RPU no longer consumes a machine-configuration artifact or generated
BSP-domain headers. A full workspace build is:

```sh
./make_RPU.sh
```

After the platform has been created, source-only firmware changes can reuse
it when the XSA and contract digests still match:

```sh
./make_RPU.sh --elf-only
```

The generated headers are installed temporarily at:

```text
runtime-generated/openamp_contract/r5c0/openamp_contract.h
runtime-generated/openamp_contract/r5c1/openamp_contract.h
```
