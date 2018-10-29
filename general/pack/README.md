# Problem

N input channels are presented as an input on each cycle. Each input
channel may or may not carry valid data. At the output, pack the M
valid inputs so that they are located in contiguous output channels
(where M <= N). In order words, rearrange/shuffle the input data such
that all non-valid channels are packed towards the MSB.

# Commentary

There are two aspects to this problem. Firstly, for each input
channel, the corresponding output channel should be computed. It can
be noted that for a given input channel A, the corresponding output
channel is (also) A, if and only if the preceeding channels are
valid. Otherwise the output channel is A - offset, where offset is the
number of non-valid locations preceeding A. For each input, the
corresponding output channel can be computed by performing a
population count of the non-valid channels.

The second aspect of the problem involves the permutation. Once the
mapping between input to output channel has been computed, the output
can be derived by use of a simple crossbar-like structure connecting
each output B, to each input A (for all A >= B). A more sophisticated
structure such as a Benes network can be employed for large N.

The final permutation can also be considered to be a sort operation,
where the sort is performed on the output index derived by the
population count. For large N, it may be necessary to pipeline the
shuffle operation to achieve timing; although this is not demonstrated
here.
