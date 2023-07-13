<!-- SPDX-License-Identifier: MIT -->
<!--
MIT License

Copyright (c) 2023 Alexander F. Lent

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice (including the next paragraph) shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
-->
# Linux kernel for the Google Pixel Watch derived from AOSP

This project exists to streamline the process of compiling Linux kernels for the Google Pixel Watch ([r11/r11btwifi](https://developers.google.com/android/images-watch) a.k.a. [rohan](https://groups.google.com/g/android-building/c/ALQP58NCpEM)) from AOSP sources.

This kernel should work for both models of the device, variously called [r11, Google Pixel Watch (LTE), r11btwifi, Google Pixel Watch (Bluetooth/Wi-Fi)](https://developers.google.com/android/images-watch), [Google Pixel Watch LTE, Google Pixel Watch Wi-Fi](https://support.google.com/googlepixelwatch/answer/12674814?hl=en&ref_topic=12652267), or [just Google Pixel Watch with the option to "Choose connectivity" between "Bluetooth®/Wi-Fi", and "4G LTE + Bluetooth®/Wi-Fi"](https://store.google.com/product/google_pixel_watch?hl=en-US).

It attempts to combine the multiple source repositories used by AOSP into a single repository with minimal differences to AOSP, to allow an Android-like kernel to be built without using Android's kernel build system.

For more information on the content of this release, please reference the [Release Information](#release-information) and [Upstream AOSP Sources](#upstream-aosp-sources) sections.

For information about this project, please see the [About "`COMBINED_`" Branches &amp; Tags](#about-combined_-branches--tags) and [Background](#background) sections.

## Release Information

__WARNING:__ This kernel _has not been tested on actual hardware._  
It is provided with __ABSOLUTELY NO WARRANTY__ and you use it __AT YOUR OWN RISK__.

This kernel release is: `COMBINED_REV0_android-wear-11.0.0_r0.11`

This is the first combined kernel release for the Google Pixel Watch (r11/r11btwifi) based on AOSP sources for `android-wear-11.0.0_r0.11` ("Android wear 11.0.0 release 0.11").

This kernel is based on Linux v4.19.279 and the Android Common Kernel v4.19. It incorporates the kernel content of Android Security Patch Level 2023-04-05 and earlier via Android Common Kernel tag `ASB-2023-04-05_4.19-stable` as well as assorted security fixes for the ARM Mali Midgard GPU driver.

### Change Log

This is the second overall release of this project.

- `COMBINED_REV0_android-wear-11.0.0_r0.11`: Second "`COMBINED`" project release. First release based on AOSP `android-wear-11.0.0_r0.11`, the July 2023 GPL software release for the Google Pixel Watch.
	- Upstream AOSP changes from `android-wear-11.0.0_r0.10`:
		- A single commit fixing an issue in the `kernel/exynos` repository: `07e8f4e40feb Revert "Revert "mm/rmap: Fix anon_vma->degree ambiguity leading to double-reuse""`
		- No changes to each of the three `kernel/exynos-modules`.
	- Downstream changes from `COMBINED_REV0_android-wear-11.0.0_r0.10`:
		- Updates to README.md.
- `COMBINED_REV0_android-wear-11.0.0_r0.10`: First "`COMBINED`" project release. First release based on AOSP `android-wear-11.0.0_r0.10`, the June 2023 GPL software release for the Google Pixel Watch.

## Upstream AOSP Sources

This kernel was created by merging together the branch named `android-exynos-r11-4.19-android11-wear-jr3-qpr2-dr` when its head was at the commit tagged `android-wear-11.0.0_r0.11` in each of the four principal Linux kernel source repositories for Samsung Exynos SoCs in AOSP:  
(The Google Pixel Watch uses a Samsung Exynos 9110 SoC, which is designed for smartwatches.)

- [`android/kernel/exynos`](https://android.googlesource.com/kernel/exynos/+/refs/tags/android-wear-11.0.0_r0.11)
- [`android/kernel/exynos-modules/config`](https://android.googlesource.com/kernel/exynos-modules/config/+/refs/tags/android-wear-11.0.0_r0.11)
- [`android/kernel/exynos-modules/devicetree`](https://android.googlesource.com/kernel/exynos-modules/devicetree/+/refs/tags/android-wear-11.0.0_r0.11)
- [`android/kernel/exynos-modules/drivers`](https://android.googlesource.com/kernel/exynos-modules/drivers/+/refs/tags/android-wear-11.0.0_r0.11)

First, the `COMBINED_...` branch was reset to the target branch of `android/kernel/exynos`, the main kernel tree for the device.

The `config` and `devicetree` repositories were merged in following the file renames specified in the [corresponding AOSP kernel manifest](https://android.googlesource.com/kernel/manifest/+/refs/tags/android-wear-11.0.0_r0.6/default.xml). (Note that Google has not released a kernel manifest corresponding to the tag used, so an older manifest version was used instead.) Additionally, two commits ignoring these files in the main tree were reverted and two minor were fixes applied to make the kernel compile more convenient on typical distributions.

The largest and final change is the merge of the out-of-tree `drivers`. In AOSP, these modules are compiled outside the kernel tree. They were merged into the kernel tree to make compiling a useful kernel simpler, but compiling modules in-tree creates some deviation in output files from AOSP.

Since the changes to the previous downstream version were minimal, all of the downstream non-merge commits were either cherry-picked to save time or trivially recreated.

## About "`COMBINED_`" Branches &amp; Tags
Anyone using this repository should use the tags. Tags identify releases that correspond to code derived from upstream AOSP tagged releases. Branches are used only for development purposes.

### Tags
Tags have names like `COMBINED_REVn_upstream-tag_name`, where `COMBINED_REVn_` is a prefix and `n` is an integer \>\= 0 indicating the downstream revision. The integer has no limit on size or length, and is incremented with each downstream revision. The downstream revision differentiates downstream releases based on the same `upstream-tag_name`.

### Branches
Branches have names like `COMBINED_YYYYMMDD_upstream-branch-name`, where `COMBINED_YYYYMMDD_` is a prefix and `YYYYMMDD` is the approximate date (in year month day format) that the downstream branch was created from `upstream-branch-name`.

## Background
### Overview of Linux kernels in AOSP

The Android Open Source Project (AOSP) provides Linux kernel source code for many different Android-based devices, in addition to the source code for the Android Operating System.

AOSP uses a tool called [repo](https://gerrit.googlesource.com/git-repo/+/HEAD/README.md) to manage hundreds of git repositories in a hierarchy specified by a "manifest". Each repository contains a separate part of the OS source code. The manifest will specify a branch or a tag to check out in each of the repositories. AOSP identifies Android OS versions by branches and identifies numbered releases by tags. Numbered releases are device-specific: An Android OS release may correspond to several AOSP numbered releases, one or more for each device. [See the Android Source documentation on Codenames, Tags, and Build Numbers for more information.](https://source.android.com/docs/setup/about/build-numbers#source-code-tags-and-builds)
 
The Android OS build process separates the Linux kernel build from the Android OS build: The Android OS build process uses kernel binaries ("prebuilts") checked into version control. When kernel updates are required, the kernel is built separately and the prebuilts are checked in. This means that one can build the Linux kernel without worrying about the rest of the Android OS. Android kernel repositories are specific to a particular SoC vendor. Unlike in the rest of AOSP, branches specify the SoC vendor name and the name of the device in addition to the Android OS version corresponding to the branch. Tags correspond with the rest of AOSP and identify releases by number. [See the Android Source documentation on Building Kernels for more information about the Android way to build kernels.](https://source.android.com/docs/setup/build/building-kernels)

Building Linux kernels from AOSP used to be straightforward: Check out the relevant kernel sources by choosing a branch or a tag from an SoC-specific kernel source repository, then build it with the kernel's Makefile-based build system. AOSP has moved away from self-contained kernel trees and the kernel's Makefile-based build system. Now, kernel sources are assembled from multiple repositories using a repo manifest. Typically this consists of an SoC vendor repository, some device specific configs/devicetrees/drivers, and various drivers shared between multiple devices. In addition to the kernel sources, the build process uses an Android-specific build mechanism (build.sh or Bazel) and prebuilt compiler and tool binaries (intended to make kernel builds reproducible). These changes are frustrating for those of us trying to build kernels with Makefiles, but understandable from an engineering perspective, since downstream kernel components are easier to maintain separately: Rather than merging security patches into every device-specific kernel tree every month, now they can be applied to common trees and reused between devices.

Finally, merging AOSP git repositories together presents a challenge when using Git: AOSP and other repo-based projects use the same tag and branch names in different repositories to refer to unrelated histories, since the repositories are meant to be combined at a file system level, not a version control level. Git can handle having multiple remote-tracking branches with the same name, since they are namespaced to an origin, but Git uses a global tag namespace that leads to troublesome tag name collisions between different histories when multiple AOSP repositories are naively combined. The solution used to build this repository was to use `git config remote.<remote_name>.tagOpt --no-tags` to prevent conflicting tags from being downloaded by default, and when referencing tags was necessary, to use `git ls-remote` or the AOSP Gitiles web interface.

### Motivation
#### Using Android kernels in Linux distributions
While the Android kernel build process is well-documented and functional, it does not integrate well with Linux distribution packaging. Most Linux distribution maintainers prefer configuring, building and packaging their own kernels, and would object to using binary kernel releases or the Android build system.

#### Linux distributions target phones
In recent years, mobile devices have become much more powerful and capable computers, and Linux [distributions](https://postmarketos.org/) [targeting](https://mobian-project.org/) [mobile](https://mobile.nixos.org/) [devices](https://pureos.net/) [have](https://ubports.com/en/) [proliferated](https://manjaro.org/). Thanks to a great deal of community effort, [mobile-first](https://honk.sigxcpu.org/con/phosh_overview.html) [graphical](https://plasma-mobile.org/) [user](https://blogs.gnome.org/shell-dev/2022/09/09/gnome-shell-on-mobile-an-update/) [interfaces](https://sxmo.org/) are now available. There are even [distributions](https://wiki.postmarketos.org/wiki/Category:Watch) [for smart watches](https://asteroidos.org/).

#### Android hardware is a difficult target
These distributions typically work best on well-documented Linux-specific hardware designed to empower fully open source distributions. Devices like the [Purism Librem 5](https://puri.sm/products/librem-5/) and the Pine64 [PinePhone](https://www.pine64.org/pinephone/) and [PinePhone Pro](https://www.pine64.org/pinephonepro/) provide an affordable, well-supported hardware platform, which has fostered the creation of mobile interfaces and apps for Linux.

Comparatively, Android hardware is poorly documented, full of proprietary firmware and software components, hard for end users to recover to the stock OS, and sometimes requires fighting manufacturers for kernel source code. Nevertheless, a few dedicated volunteers maintain support for a handful of Android devices in mobile distributions.

#### Google provides infrastructure for users and developers

For Google Pixel (and older Nexus) devices, Google provides:

- Hardware where the bootloader can be unlocked and third-party OSes installed on the device.
- Source code via AOSP for phones and tablets. Some proprietary components are omitted, but the open-source code is still a valuable resource (the exact location of the notch in the screen, code for some hardware drivers in userspace, etc.).
- Source code via AOSP for watches, but only the kernel.
- Factory Images (and the Android Flash Tool) to return a modified device to a supported state. (Modulo users wiping partitions containing critical components or vital product data, or damaging hardware with a bad driver.)
- Vendor Binaries for use with AOSP. These and Factory Images are a useful source of proprietary bits needed to make the device work.
- Sideload-able Full OTA images as an alternative format for updating device software.
- A repair/calibration tool for Pixel phones with an in-screen fingerprint reader.

This is substantially more access to code, tools, and documentation than other vendors. Hopefully, that access and work like this project make it easier for distributions to maintain support for Pixel devices.

### Project Goals
The goal of this project is to make is easier to support various Google Pixel devices in Linux distributions for mobile devices.

This goal is achieved by providing AOSP-dervied kernels suitable for porting distributions to Android hardware. Kernels they can patch, package, and build as they usually do, without dealing with the Android build system.

More specifically, this goal is achieved by combining the various AOSP source repositories into a single repository that is easier to build and use.

### Starting with the Pixel Watch
It was unclear to the Author how much work would be required to combine a "modern" Android kernel into a single repository that could be built and used by more traditional Linux distributions, but it seemed possible.

Compared to other current Android kernels, the kernel for the Google Pixel Watch ("r11") was interesting because it had only four principal source repositories, and did not use the latest Android-specific build system, potentially reducing the work required to combine them.  (The Pixel Watch kernel is actually quite old; it is based on the Android v4.19 Stable Kernel.)

Additionally, the Google Pixel Watch was interesting because, as far as the Author is aware, it is the only Android Wear/Wear OS device to have [publicly-available Factory Images](https://developers.google.com/android/images-watch) and [Full OTA images](https://developers.google.com/android/ota-watch), which make it uniquely easy to recover Wear OS after using a third-party OS.

Like some (but not all) other Android Wear/Wear OS devices, it has a USB connector that is externally accessible. Google's documentation says that this connector requires "a debug adapter that Google distributes by invitation only" but as with other Android-based watches, hobbyists should be able to substitue offical development hardware with some pogo pins and a 3d-printed cradle, since the protocol is just USB. The important part is that you can develop on this watch without damaging the water/dust resistance.

