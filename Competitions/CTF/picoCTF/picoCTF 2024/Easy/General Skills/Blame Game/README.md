# Blame Game

## How to resolve?

````bash
$ unzip challenge.zip
````

Extract files and directories seem to be local git repository.
And [drop-in/message.py](./drop-in/message.py) is broken.

Problem says `Someone's commits seems to be preventing the program from working. Who is it?`.
So you can check git log by `git log -p message.py`.

In other words?