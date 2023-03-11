# Check whitespace

## Abstracts

* Analysis C++ source code about

## Dependencies

* [cpplint](https://github.com/cpplint/cpplint)
  * 1.6.1
  * 3-Clause BSD License

## What's are checked?

|Rule|Description|Description (Japanese)|
|---|---|---|
|whitespace/blank_line|* Redundant blank line at the start of a code block should be deleted.<br>* Redundant blank line at the end of a code block should be deleted.<br>* Do not leave a blank line after `public:`, `protected:` or `private:`<br>* `public:`, `protected:` or `private:` should be preceded by a blank line|* コードブロックの開始行にある冗長な空白行を削除するべき<br>* コードブロックの終了行にある冗長な空白行を削除するべき<br>* `public:`, `protected:` または `private:` の後に空白行を残さない<br>* `public:`, `protected:` または `private:` には空白行を先行させること|
|whitespace/braces|* Extra space before `[`<br>* Missing space before `{`<br>* Missing space before `else`<br>* `{` should almost always be at the end of the previous line|* `[` の前にスペースを追加すること<br>* `{` の前に空白が見つかならない<br>* `else` の前に空白が見つかならない<br>* `{` は常に前の行の終端にあること|
|whitespace/comma|Missing space after `,`|`,` の後に空白が見つからない|
|whitespace/comments|* At least two spaces is best between code and comments<br>* Should have a space between `//` and comment|* コードとコメントの間には最低でも 2 つの空白があることが適切<br>* `//` とコメントの間には空白が必要|
|whitespace/empty_conditional_body|Empty conditional bodies should use `{}`|空の条件のコード部には `{}` を使うべき|
|whitespace/empty_if_body|If statement had no body and no else clause|`If` 文に本文と `else` 句がない|
|whitespace/empty_loop_body|Empty loop bodies should use `{}` or continue|空のループボディは `{}` または `continue` を使用します|
|whitespace/end_of_line|Line ends in whitespace. Consider deleting these extra spaces.|行末に空白があります。これらの余分な空白を削除することを検討してください|
|whitespace/ending_newline|Could not find a newline character at the end of the file.|ファイルの末尾に改行文字が見つかりませんでした|
|whitespace/forcolon|Missing space around colon in range-based for loop|範囲型 `for` ループでコロンの周りにスペースがない|
|whitespace/indent|Weird number of spaces at line-start. Are you using a 2-space indent?|行頭のスペース数がおかしい。2 スペースのインデントを使用していますか？|
|whitespace/line_length|Lines should be <= N characters long|行の長さは N 文字以下であること|
|whitespace/newline|* An `else` should appear on the same line as the preceding `}`<br>* `Else` clause should never be on same line as `else` (use 2 lines)<br>* `do/while` clauses should not be on a single line<br>* More than one command on the same line<br>* Unexpected `\r` (^M) found; better to use only `\n`|* `else` は、直前の `}` と同じ行に記述します<br>* `else` 句は `else` と同じ行には絶対に書かないこと (2 行使う)<br>* `do/while` 句は、1 行にまとめてはならない<br>* 同じ行に 2 つ以上の命令がある<br>* 予期せぬキャリッジリターン (`\r`) が⾒つかりました。ラインフィード (`\n`) だけを使うべきです|
|whitespace/operators|* Missing spaces around `=`<br>* Missing spaces around %s<br>* Missing spaces around `<<`<br>* Missing spaces around `>`<br>* Missing spaces around `>>`<br>* Extra space for operator %s|* `=` の周りにスペースがありません<br>* %s の周りにスペースがありません<br>* `<<` の周りにスペースがありません<br>* `>` の周りに空白がありません<br>* `>>` の周りにスペースがありません<br>* 演算子 %s に余分なスペースがあります|
|whitespace/parens|* Extra space after ( in function call<br>* Extra space after (<br>* Extra space before ( in function call<br>* Closing `)` should be moved to the previous line<br>* Extra space before `)`<br>* Missing space before `(` in %s<br>* Mismatching spaces inside `()` in %s<br>* Should have zero or one spaces inside `(` and `)` in %s|* 関数呼び出しの `()` の後に余分なスペースがあります<br>* `()` の後に余分なスペースがある。<br>* 関数呼び出しで、`()` の前に余分なスペースがある。<br>* クロージングの `)` を前の行に移動する必要があります<br>* `)` の前に余分なスペースがあります<br>* %s の中の `()` の前にスペースがありません<br>* %s の中の `()` 内部のスペースが一致しません<br>* %s の中の `(` と `)` 内部のスペースは 0 か 1 であること|
|whitespace/semicolon|* Missing space after `;`<br>* Semicolon defining empty statement. Use `{}` instead.<br>* Line contains only semicolon. If this should be an empty statement, use `{}` instead.<br>* Extra space before last semicolon. If this should be an empty statement, use `{}` instead.|* `;` の後にスペースがありません<br>* セミコロンで空の文を定義しています。代わりに `{}` を使用します。<br>* 行にはセミコロンのみが含まれます。もしこれが空の文であるべきなら、代わりに `{}` を使します。<br>* 最後のセミコロンの前に余分なスペースがあります。もしこれが空の文であるべきなら、代わりに `{}` を使います|
|whitespace/tab|Tab found; better to use spaces|タブが見つかりました。スペースを使用するべきです|
|whitespace/todo|* Too many spaces before `TODO`<br>* `TODO(my_username)` should be followed by a space|* `TODO` の前にスペースが多すぎる<br>* `TODO(my_username)` の後にスペースを入れてください。|

## How to use?

````sh
$ cpplint --filter=+whitespace,-legal sample.cpp 
sample.cpp:4:  { should almost always be at the end of the previous line  [whitespace/braces] [4]
sample.cpp:5:  Missing spaces around <<  [whitespace/operators] [3]
sample.cpp:7:  Could not find a newline character at the end of the file.  [whitespace/ending_newline] [5]
Done processing sample.cpp
Total errors found: 3
````

````cpp
#include <iostream>

void main()  // should almost always be at the end of the previous line
{
    std::cout<<"Hello world!!" << std::endl;  // Missing spaces around <<
}
// Could not find a newline character at the end of the file
````