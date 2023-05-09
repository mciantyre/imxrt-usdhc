# imxrt-usdhc

A driver for the i.MX RT SD/SDIO/MMC peripheral, uSDHC. Although it's designed
for the imxrt-rs project, this driver may be portable to other systems.

This driver has been tested with one SD card on one board. When paired with
[`embedded-sdmmc`], the driver was able to detect a FAT32 filesystem, create a
file in the root directory, and write a message to that file.

[`embedded-sdmmc`]: https://docs.rs/embedded-sdmmc/0.4.0/embedded_sdmmc/

Test card: SDHC, class 10, 8GB, UHS-I. Test board: iMXRT1170EVK.

This driver is incomplete. It relies on large changes to upstream projects, and
those changes are also incomplete.

## Try it

Add this package as a git dependency. Additionally, patch [`sdio-host`]
with the changes in my fork. To play with the FAT32 capabilities, also
include [`embedded-sdmmc`].

[`sdio-host`]: https://docs.rs/sdio-host/0.9.0/sdio_host/

```toml
# Cargo.toml

[dependencies.imxrt-usdhc]
git = "https://github.com/mciantyre/imxrt-usdhc"

[patch.crates-io.sdio-host]
git = "https://github.com/mciantyre/sdio-host"

[dependencies.embedded-sdmmc]
version = "0.4"
```

Before constructing the `uSDHC` driver, mux and configure uSDHC pins with your
chip's IOMUXC. In particular, this driver assumes that the uSDHC reset line is
muxed, and that it is connected to your hardware.

Enable the uSDHC clock, and configure the uSDHC clock to 400 KHz or less. You
can achieve this using just the CCM, or a combination of the CCM and uSDHC
prescalers.

Once that's all ready, create a `Usdhc` driver with a pointer to your uSDHC
peripheral memory. Then, wrap that in a `BlockingSdioHost`.

```rust
use imxrt_usdhc::{Usdhc, BlockingSdioHost};

// TODO use IOMXUC to mux pins...
// TODO use CCM to enable, configure uSDHC clock...

let mut usdhc = unsafe { Usdhc::from_instance(MY_USDHC1_PTR) };
// TODO set prescalers for uSDHC clock...

let host = BlockingSdioHost::new(usdhc).unwrap();
```

`BlockingSdioHost` initializes your card so that you can perform I/O. If that
succeeds, the host can give you information about your card.

```rust
log::info!("RCA: {:#06X}", host.rca().address());
log::info!("{:?}", host.cid());
log::info!("{:?}", host.csd());
log::info!("{:?}", host.scr());
log::info!("{:?}", host.sd_status());
```

From there, you can convert the host into a `BlockDevice` for `embedded-sdmmc`,
and you can interact with your FAT16/32-formatted SD card.

```rust
use embedded_sdmmc::Controller;
let mut ctrl = Controller::new(host.into_sdmmc_block_dev(), MyTimeSource);
// See embedded-sdmmc docs for more information.
```
