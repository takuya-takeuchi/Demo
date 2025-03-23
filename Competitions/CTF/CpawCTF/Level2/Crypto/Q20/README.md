# Q20.[Crypto]Block Cipher

## How to resolve?

At first, you must compile this source code.

````bash
$ gcc crypto100.c   
````

You will see `a.out`.

Next, you must find key.
Enrypted word is `ruoYced_ehpigniriks_i_llrg_stae`.
You will see 

````c
for(i = key - 1; i <= strlen(flag); i+=key)
````

It means that, `key` should be from 1 and `strlen(flag)`.
So you can find 

````bash
$ ./a.out ruoYced_ehpigniriks_i_llrg_stae 1
$ ./a.out ruoYced_ehpigniriks_i_llrg_stae 2
...
````

In other words?