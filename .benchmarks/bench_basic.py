def test_benchmark_basic(benchmark):
    def run():
        pass
    benchmark(run)
