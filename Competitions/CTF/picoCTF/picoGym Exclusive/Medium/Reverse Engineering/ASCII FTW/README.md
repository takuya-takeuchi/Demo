# LASCII FTW

## How to resolve?

You will see message `The flag starts with 70` by running [asciiftw](asciiftw).
And this problem says `This program has constructed the flag using hex ascii values.`.
So `70` could be hex ascii value.

And, you can use `objdump` to see content of object file.
So you will find that there is strange code in main section.

In other words?