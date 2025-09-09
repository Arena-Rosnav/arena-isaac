def safe():
    def inner(fun):
        def wrapper(request, response):
            try:
                return fun(request, response)
            except Exception as e:
                import sys
                print(f"Error in {fun.__name__}: {e}", file=sys.stderr)
                response.ret = False
            return response
        return wrapper
    return inner
