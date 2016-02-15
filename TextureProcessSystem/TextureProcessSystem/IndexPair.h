class IndexPair
{
public:
	int sampleIndex;
	int targetIndex;
public:
bool IndexPair::operator == (const int index) const
{
return (targetIndex == index);
}
};