#include "gtest/gtest.h"
#include "simpleQueue.h"
TEST(simpleQueueuTest, doSimpleQueueTest)
{
	SimpleQueue *sq = NULL;
	addToQueue(&sq, (char *)"test");
	addToQueue(&sq, (char *)"test2");
	ASSERT_STREQ(retrieveFromQueue(&sq), (char *)"test");
	ASSERT_STREQ(retrieveFromQueue(&sq), (char *)"test2");
	ASSERT_STREQ(retrieveFromQueue(&sq), (char *)"e:e");
}

TEST(simpleQueueuTest, addToQueue)
{
	SimpleQueue *sq = NULL;
	ASSERT_EQ((sq), nullptr);
	addToQueue(&sq, (char *)"test");
	ASSERT_NE((sq), nullptr);
	//test from first is "test"
	ASSERT_EQ(sq->value, (char *)"test");
	ASSERT_EQ(sq->nextSQ, nullptr);

	addToQueue(&sq, (char *)"test2");
	ASSERT_NE(sq->nextSQ, nullptr);
	ASSERT_EQ(sq->nextSQ->value, (char *)"test2");

	//destroy
	retrieveFromQueue(&sq);
	retrieveFromQueue(&sq);
}

TEST(simpleQueueuTest, retrieveFromEmptyQueue)
{
	SimpleQueue *sq = NULL;
	ASSERT_STREQ(retrieveFromQueue(&sq), (char *)"e:e");
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}