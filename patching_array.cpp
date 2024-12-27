#include <iostream>
#include <vector>

int minPatches(std::vector<int> &nums, int n)
{
    long long sum = 0;
    int i = 0, size = nums.size(), ans = 0;

    while (sum < n && i < size)
    {
        if (nums[i] > sum + 1)
        {
            sum += sum + 1;
            ans++;
        }
        else
        {
            sum += nums[i];
            i++;
        }
    }

    while (sum < n)
    {
        sum += sum + 1;
        ans++;
    }

    return ans;
}

int main()
{
    std::vector<int> nums = {1, 5, 10};
    int n = 20;
    int result = minPatches(nums, n);
    std::cout << "Минимальное количество: " << result << std::endl;
    return 0;
}