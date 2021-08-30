# gift
Hardware implementation of the GIFT-128 lightweight block cipher

## Status
Just started. Not completed. Does not work. **Do Not Use**.


## Introduction
[GIFT-128 (PDF)](https://eprint.iacr.org/2017/622.pdf) is a round based,
lightweight block cipher. The key length and block size is 128 bits.

Verification is done against [the official GIFT test
vectors](https://github.com/giftcipher/gift).


## FuseSoC
This core is supported by the
[FuseSoC](https://github.com/olofk/fusesoc) core package manager and
build system. Some quick  FuseSoC instructions:

Install FuseSoC
~~~
pip install fusesoc
~~~

Create and enter a new workspace
~~~
mkdir workspace && cd workspace
~~~

Register prince as a library in the workspace
~~~
fusesoc library add gift /path/to/gift
~~~
...if repo is available locally or...
...to get the upstream repo
~~~
fusesoc library add gift https://github.com/secworks/gift
~~~

Run tb_prince testbench
~~~
fusesoc run --target=tb_gift secworks:crypto:gift
~~~

Run with modelsim instead of default tool (icarus)
~~~
fusesoc run --target=tb_gift --tool=modelsim secworks:crypto:gift
~~~
