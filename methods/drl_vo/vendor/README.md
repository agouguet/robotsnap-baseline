# Vendored DRL-VO pieces

`custom_cnn_full.py` is a verbatim copy of
`drl_vo_nav/drl_vo/src/custom_cnn_full.py` from
[TempleRAIL/drl_vo_nav](https://github.com/TempleRAIL/drl_vo_nav) (MIT licence,
see that repository). It holds the `CustomCNN` feature extractor the checkpoint
was trained with, and it is copied rather than reimplemented so the network
stays byte-for-byte the one the published policy expects.

`gym/` is a small shim. The DRL-VO code was written against `gym` 0.18, whose
`gym.spaces` package the checkpoint's pickled observation space still names.
This host only ships `gymnasium`, so the shim re-exports the gymnasium classes
under the old `gym` names. It carries no behaviour of its own.
