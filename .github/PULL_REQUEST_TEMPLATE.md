# Pull Request

## Description
Brief description of the changes made.

## Type of Change
- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation update
- [ ] TDD implementation (new tests added following t-wada principles)

## TDD Checklist
- [ ] Tests were written before implementation (RED phase)
- [ ] Implementation makes tests pass (GREEN phase) 
- [ ] Code was refactored for maintainability (REFACTOR phase)
- [ ] All existing tests still pass
- [ ] New tests cover edge cases and error conditions
- [ ] Test coverage is maintained or improved

## Testing
- [ ] All tests pass on PC build (`./build_pc.sh`)
- [ ] STM32 build compiles successfully
- [ ] Code follows formatting standards (clang-format)
- [ ] No static analysis warnings introduced

## Checklist
- [ ] My code follows the style guidelines of this project
- [ ] I have performed a self-review of my own code
- [ ] I have commented my code, particularly in hard-to-understand areas
- [ ] I have made corresponding changes to the documentation
- [ ] My changes generate no new warnings
- [ ] Any dependent changes have been merged and published