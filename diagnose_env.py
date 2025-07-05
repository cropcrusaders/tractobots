import sys
import os

print("--- Python Environment Diagnostic ---")

try:
    print(f"Python Version: {sys.version}")
    print(f"Python Executable: {sys.executable}")
    print(f"Current Working Directory: {os.getcwd()}")
    print("\n--- Checking Dependencies ---")

    dependencies = ['PyQt5', 'numpy', 'matplotlib', 'roslibpy']
    for lib in dependencies:
        try:
            __import__(lib)
            print(f"✅ Successfully imported '{lib}'")
        except ImportError as e:
            print(f"❌ FAILED to import '{lib}': {e}")
        except Exception as e:
            print(f"❓ UNEXPECTED ERROR importing '{lib}': {e}")

    print("\n--- Diagnostic Complete ---")

except Exception as e:
    print(f"\nAn unexpected error occurred during diagnostics: {e}")

finally:
    print("\nPlease copy all the text above and paste it in the chat.")
    input("Press Enter to exit...")
