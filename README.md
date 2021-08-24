# gift
Hardware implementation of the GIFT-128 lightweight block cipher

## Status
Just started. Not completed. Does not work. **Do Not Use**.


## Introduction
[GIFT-128 (PDF)](https://eprint.iacr.org/2017/622.pdf) is a round based,
lightweight block cipher. The key length and block size is 128 bits.



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
fusesoc library add prince /path/to/prince
~~~
...if repo is available locally or...
...to get the upstream repo
~~~
fusesoc library add prince https://github.com/secworks/prince
~~~

Run tb_prince testbench
~~~
fusesoc run --target=tb_prince secworks:crypto:prince
~~~

Run with modelsim instead of default tool (icarus)
~~~
fusesoc run --target=tb_prince --tool=modelsim secworks:crypto:prince
~~~
