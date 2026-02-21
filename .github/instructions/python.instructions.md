---
applyTo: "**/*.{py}"
---

Write Python docstrings in Sphinx style and always use type hints for the code.

**Language:** Use **American English** spelling in all docstrings, comments, and string literals.
- `color` not `colour`, `gray` not `grey`, `initialize` not `initialise`, `optimize` not `optimise`,
  `normalize` not `normalise`, `recognize` not `recognise`, `visualize` not `visualise`, etc. Here are some examples:

```python
from typing import Dict, List, Union, Optional, Any, Generator, AsyncGenerator
import asyncio

def example_function(param1: int, param2: str) -> bool:
    """
    Example function that does something.

    :param param1: Description of the first parameter.
    :type param1: int
    :param param2: Description of the second parameter.
    :type param2: str
    :return: Description of the return value.
    :rtype: bool
    """
    return True

def process_data(data: List | Dict, config: Optional[Dict] = None) -> Dict:
    """
    Process input data according to configuration.

    :param data: The input data to process.
    :type data: list or dict
    :param config: Optional configuration dictionary.
    :type config: dict, optional
    :return: Processed data.
    :rtype: dict
    :raises ValueError: If data format is invalid.
    :raises TypeError: If config is not a dictionary.
    """
    if config is None:
        config = {}
    return {"processed": data, "config": config}

def calculate_average(*args: float | int, **kwargs: Any) -> float:
    """
    Calculate the average of provided numbers.

    :param args: Variable number of numeric arguments.
    :type args: float or int
    :param kwargs: Optional keyword arguments for configuration.
    :type kwargs: dict
    :keyword precision: Number of decimal places (default: 2).
    :type precision: int
    :keyword exclude_zeros: Whether to exclude zero values (default: False).
    :type exclude_zeros: bool
    :return: The calculated average.
    :rtype: float
    :raises ZeroDivisionError: If no valid numbers provided.
    """
    precision = kwargs.get('precision', 2)
    exclude_zeros = kwargs.get('exclude_zeros', False)

    numbers = [x for x in args if not (exclude_zeros and x == 0)]
    if not numbers:
        raise ZeroDivisionError("No valid numbers provided")

    return round(sum(numbers) / len(numbers), precision)

async def fetch_data(url: str, timeout: int = 30) -> Dict:
    """
    Asynchronously fetch data from a URL.

    :param url: The URL to fetch data from.
    :type url: str
    :param timeout: Request timeout in seconds.
    :type timeout: int, optional
    :return: The fetched data.
    :rtype: dict
    :raises aiohttp.ClientError: If the request fails.
    :raises asyncio.TimeoutError: If the request times out.
    """
    # Implementation would go here
    pass

class DataProcessor:
    """
    A class for processing various types of data.

    :param name: The name identifier for this processor.
    :type name: str
    :param options: Processing options.
    :type options: dict, optional

    .. note::
       This class is designed to handle multiple data formats.

    .. example::
       >>> processor = DataProcessor("my_processor")
       >>> result = processor.process([1, 2, 3])
    """

    def __init__(self, name: str, options: Optional[Dict] = None) -> None:
        """
        Initialize the DataProcessor.

        :param name: The name identifier for this processor.
        :type name: str
        :param options: Processing options.
        :type options: dict, optional
        """
        self.name = name
        self.options = options or {}

    def process(self, data: List| Dict | str) -> Dict:
        """
        Process the input data.

        :param data: Data to be processed.
        :type data: list, dict, or str
        :return: Processed data.
        :rtype: dict
        :raises NotImplementedError: If data type is not supported.
        """
        if isinstance(data, list):
            return {"type": "list", "count": len(data), "data": data}
        elif isinstance(data, dict):
            return {"type": "dict", "keys": list(data.keys()), "data": data}
        else:
            raise NotImplementedError(f"Data type {type(data)} not supported")

    @property
    def status(self) -> str:
        """
        Get the current status of the processor.

        :return: Current status.
        :rtype: str
        """
        return "ready"

    @staticmethod
    def validate_input(data: Any) -> bool:
        """
        Validate input data format.

        :param data: Data to validate.
        :type data: any
        :return: True if valid, False otherwise.
        :rtype: bool

        .. note::
           This is a static method and doesn't require an instance.
        """
        return data is not None

    @classmethod
    def from_config(cls, config: Dict) -> 'DataProcessor':
        """
        Create a DataProcessor instance from configuration.

        :param config: Configuration dictionary.
        :type config: dict
        :return: New DataProcessor instance.
        :rtype: DataProcessor
        :raises KeyError: If required configuration keys are missing.
        """
        return cls(config["name"], config.get("options"))

def generator_example(start: int, end: int, step: int = 1) -> Generator[int, None, None]:
    """
    Generate a sequence of numbers.

    :param start: Starting value.
    :type start: int
    :param end: Ending value (exclusive).
    :type end: int
    :param step: Step size.
    :type step: int, optional
    :yield: Next number in sequence.
    :rtype: int

    .. example::
       >>> list(generator_example(0, 5, 2))
       [0, 2, 4]
    """
    current = start
    while current < end:
        yield current
        current += step

async def async_generator_example(items: List[Any]) -> AsyncGenerator[Any, None]:
    """
    Asynchronously yield items from a list.

    :param items: List of items to yield.
    :type items: list
    :yield: Next item in the list.
    :rtype: any

    .. note::
       This is an async generator function.
    """
    for item in items:
        await asyncio.sleep(0.1)  # Simulate async work
        yield item

def complex_function(
    data: Dict[str, int | str | List],
    callback: Optional[callable] = None,
    options: Optional[Dict[str, Any]] = None
) -> Tuple[bool, Dict[str, Any]]:
    """
    Process complex data with optional callback and configuration.

    :param data: Complex data structure to process.
    :type data: dict
    :param callback: Optional callback function to execute.
    :type callback: callable, optional
    :param options: Optional configuration options.
    :type options: dict, optional
    :return: Tuple of success status and processed data.
    :rtype: tuple

    .. example::
       >>> result = complex_function({"key": "value"})
       >>> success, processed = result
    """
    if options is None:
        options = {}

    processed = {"input": data, "options": options}

    if callback and callable(callback):
        try:
            callback(processed)
        except Exception as e:
            return False, {"error": str(e)}

    return True, processed
```
